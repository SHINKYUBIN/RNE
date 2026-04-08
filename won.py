import os, csv, math, time
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import cv2
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from torchvision.models import resnet50, ResNet50_Weights
from ultralytics import YOLO
from scipy.optimize import linear_sum_assignment

# ===== Paths =====
MODEL_PATH  = r"/home/pi/Downloads/slippers3/best.pt"
OUT_CSV     = r"/home/pi/Downloads/Viz/match_results.csv"
OUT_VIZ_DIR = r"/home/pi/Downloads/Viz"

# ===== Settings =====
CONF_MIN = 0.60
IMG_SIZE = 800
IOU_NMS  = 0.60
ORIENT_NORMALIZE = True
SIM_MIN = 0.20
USE_PERSPECTIVE_CROP = True
USE_HUNGARIAN = True
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# [수정됨] 구멍 매칭 최대 거리 (픽셀 단위, 제곱값 비교용)
# 200픽셀 이상 떨어진 구멍은 무시 (오매칭 방지)
MAX_HOLE_DIST_SQ = 200 ** 2 

# [ROI Settings] Coordinates for the wooden board
USE_ROI_CROP = True   
ROI_X1 = 550          # Left
ROI_Y1 = 150          # Top
ROI_X2 = 1200         # Right
ROI_Y2 = 550          # Bottom

os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
os.makedirs(OUT_VIZ_DIR, exist_ok=True)

# ===== Perf Optimization =====
torch.set_grad_enabled(False)
try:
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.benchmark = True
except Exception:
    pass

# ===== Models =====
print("Loading Models...")
yolo = YOLO(MODEL_PATH)

try:
    resnet = resnet50(weights=ResNet50_Weights.DEFAULT)
except Exception:
    resnet = resnet50(weights=None)
resnet.fc = nn.Identity()
resnet.eval().to(DEVICE)

transform = transforms.Compose([
    transforms.Resize((224,224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
])

# ===== Label resolution =====
def resolve_class_ids(yolo_model, left_key='s_l', right_key='s_r', hole_key='hole'):
    names = yolo_model.model.names if hasattr(yolo_model.model, 'names') else {}
    if not isinstance(names, dict) or len(names) == 0:
        raise RuntimeError("YOLO model has no class names.")
    
    # Normalize names
    norm = {str(v).replace(' ', '').lower(): int(k) for k, v in names.items()}
    
    lk = left_key.replace(' ', '').lower()
    rk = right_key.replace(' ', '').lower()
    hk = hole_key.replace(' ', '').lower()
    
    left_id  = norm.get(lk, None)
    right_id = norm.get(rk, None)
    hole_id  = norm.get(hk, None)
    
    if left_id is None or right_id is None:
        raise RuntimeError(f"Required classes ({left_key}, {right_key}) not found. Found: {list(names.values())}")
    
    if hole_id is None:
        print(f"Warning: '{hole_key}' class not found. Hole matching will be skipped.")
        hole_id = -999 
        
    return names, left_id, right_id, hole_id

# [수정됨] 'hold' -> 'hole'로 오타 수정
names, LEFT_ID, RIGHT_ID, HOLE_ID = resolve_class_ids(yolo, 'left', 'right', 'hole')

# ===== Geometry / Cropping =====
def obb_corners_from_xywhr(cx, cy, w, h, theta_rad):
    ct, st = math.cos(theta_rad), math.sin(theta_rad)
    dx, dy = w/2.0, h/2.0
    pts = [(-dx,-dy),(dx,-dy),(dx,dy),(-dx,dy)]
    out=[]
    for x,y in pts:
        xr = x*ct - y*st + cx
        yr = x*st + y*ct + cy
        out.append((float(xr), float(yr)))
    return out

def clip_box(x1, y1, x2, y2, W, H):
    return int(max(0, x1)), int(max(0, y1)), int(min(W-1, x2)), int(min(H-1, y2))

def obb_rotated_crop(pil_img, cx, cy, w, h, theta_rad):
    img = np.array(pil_img.convert("RGB"))
    H, W = img.shape[:2]
    M = cv2.getRotationMatrix2D((cx, cy), np.degrees(theta_rad), 1.0)
    rotated = cv2.warpAffine(img, M, (W, H))
    x1 = int(cx - w/2); y1 = int(cy - h/2)
    x2 = int(cx + w/2); y2 = int(cy + h/2)
    x1, y1, x2, y2 = clip_box(x1, y1, x2, y2, W, H)
    crop = rotated[y1:y2, x1:x2]
    if crop.size == 0: return None
    return Image.fromarray(crop)

def order_corners_quad(pts):
    pts = np.array(pts, dtype=np.float32)
    c = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:,1]-c[1], pts[:,0]-c[0])
    order = np.argsort(angles)
    pts = pts[order]
    s = pts.sum(axis=1)
    top_left_idx = np.argmin(s)
    pts = np.roll(pts, -top_left_idx, axis=0)
    return pts

def obb_warped_crop(pil_img, corners, out_size=224):
    img = np.array(pil_img.convert("RGB"))
    src = order_corners_quad(corners)
    dst = np.array([[0,0],[out_size-1,0],[out_size-1,out_size-1],[0,out_size-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (out_size, out_size))
    return Image.fromarray(warped)

# ===== Shoe-Hole Matching Logic =====
def dist_sq(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def assign_holes_to_shoes(shoes_list, holes_list):
    if not shoes_list or not holes_list:
        return
    
    cost_matrix = np.zeros((len(shoes_list), len(holes_list)))
    for i, shoe in enumerate(shoes_list):
        for j, hole in enumerate(holes_list):
            # 매칭 로직은 로컬 좌표(cx,cy)를 써도 상대 거리는 같으므로 무관
            cost_matrix[i, j] = dist_sq((shoe['cx'], shoe['cy']), (hole['cx'], hole['cy']))
    
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    
    for r, c in zip(row_ind, col_ind):
        # [수정됨] 거리가 너무 멀면(MAX_HOLE_DIST_SQ 이상) 매칭하지 않음
        if cost_matrix[r, c] > MAX_HOLE_DIST_SQ:
            continue
        shoes_list[r]['hole'] = holes_list[c]

# ===== Vector Calculation =====
def calculate_shoe_vector(shoe_data):
    """
    Returns:
        fx (float): Shoe Center X (Global Coordinate)
        fz (float): Shoe Center Y (Global Coordinate)
        vec (float): Angle in radians from Shoe Center to Hole Center
    """
    # [수정됨] 벡터 계산 시 Global 좌표(gx, gy) 사용
    fx = shoe_data['gx']
    fz = shoe_data['gy']
    vec = 0.0
    
    if shoe_data['hole']:
        hx, hy = shoe_data['hole']['gx'], shoe_data['hole']['gy']
        # Vector from Shoe Center to Hole Center
        dx = hx - fx
        dy = hy - fz
        vec = math.atan2(dy, dx)
    
    return fx, fz, vec

# ===== Features / Matching =====
def get_feature_batch(crops, resnet_model, transform, device):
    if len(crops) == 0:
        return np.zeros((0, 2048), dtype=np.float32)
    with torch.inference_mode():
        batch = torch.stack([transform(c.convert("RGB")) for c in crops]).to(device)
        vecs = resnet_model(batch).detach().cpu().numpy()
    norms = np.linalg.norm(vecs, axis=1, keepdims=True) + 1e-8
    return (vecs / norms).astype(np.float32)

def cosine_sim_matrix(A, B):
    if A.shape[0] == 0 or B.shape[0] == 0:
        return np.zeros((A.shape[0], B.shape[0]), dtype=np.float32)
    return A @ B.T

def robust_match(sim, use_hungarian=True):
    if sim.size == 0: return []
    if use_hungarian:
        cost = 1.0 - sim
        r_idx, c_idx = linear_sum_assignment(cost)
        pairs = [(int(r), int(c), float(sim[r, c])) for r, c in zip(r_idx, c_idx)]
    else:
        pairs, used_cols = [], set()
        for i in range(sim.shape[0]):
            order = np.argsort(-sim[i])
            j = next((int(jj) for jj in order if sim[i, jj] > 0 and jj not in used_cols), None)
            if j is None: continue
            used_cols.add(j)
            pairs.append((i, j, float(sim[i, j])))
            
    return [(i, j, sc) for (i, j, sc) in pairs if sc >= SIM_MIN]

# ===== Drawing =====
def load_font():
    possible_fonts = ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "arial.ttf", "C:/Windows/Fonts/arial.ttf"]
    for fp in possible_fonts:
        try:
            return ImageFont.truetype(fp, 20), ImageFont.truetype(fp, 18)
        except Exception:
            continue
    f = ImageFont.load_default()
    return f, f

def draw_obb(draw, corners, color, label, score, font):
    if corners is None: return
    int_pts = [(int(x), int(y)) for (x, y) in corners]
    draw.line(int_pts + [int_pts[0]], fill=color, width=3)
    x_min = min(x for x,_ in int_pts); y_min = min(y for _,y in int_pts)
    draw.text((x_min, max(0, y_min-22)), f"{label} {score:.2f}", fill=color, font=font)

# ===== Webcam Setup =====
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# CSV Header Updated
csv_header = (
    "filename",
    "similarity",
    "R_id", "R_conf", "R_fx", "R_fz", "R_vec",
    "L_id", "L_conf", "L_fx", "L_fz", "L_vec",
    "R_w", "R_h", "R_r",
    "L_w", "L_h", "L_r"
)
rows = [csv_header]
num = 1

print("\n=== Controls ===")
print(" [c] Capture and Process (Inside ROI)")
print(" [q] Quit")
print("================\n")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed.")
        break

    vis_frame = frame.copy()
    if USE_ROI_CROP:
        cv2.rectangle(vis_frame, (ROI_X1, ROI_Y1), (ROI_X2, ROI_Y2), (0, 255, 0), 2)
        cv2.putText(vis_frame, "ROI Area", (ROI_X1, ROI_Y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    cv2.imshow('Webcam', vis_frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        if not ret: continue

        if USE_ROI_CROP:
            h, w = frame.shape[:2]
            x1 = max(0, ROI_X1)
            y1 = max(0, ROI_Y1)
            x2 = min(w, ROI_X2)
            y2 = min(h, ROI_Y2)
            process_frame = frame[y1:y2, x1:x2].copy()
            print(f"Captured ROI: {x1},{y1} to {x2},{y2}")
            
            # [좌표 보정값 설정]
            offset_x, offset_y = x1, y1
        else:
            process_frame = frame.copy()
            offset_x, offset_y = 0, 0

        if process_frame.size == 0:
            print("Error: Empty crop area.")
            continue

        img_rgb = cv2.cvtColor(process_frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(img_rgb)
        
        ts = time.strftime("%Y%m%d_%H%M%S")
        fname = f"capture_{num}_{ts}"

        print(f"\n--- Processing {fname} ---")

        res = yolo.predict(
            process_frame,
            classes=None,  
            conf=CONF_MIN,
            sz=IMG_SIZE,
            agnostic_nms=False, 
            iou=IOU_NMS,
            verbose=False
        )[0]

        if getattr(res, "obb", None) is None or len(res.obb) == 0:
            print("No OBB detections found")
            num += 1
            continue

        xywhr    = res.obb.xywhr.detach().cpu().numpy()
        cls_ids = res.obb.cls.detach().cpu().numpy().astype(int)
        confs    = res.obb.conf.detach().cpu().numpy()

        shoes_r = [] 
        shoes_l = []
        holes   = [] 

        for k, (c, cf, obb) in enumerate(zip(cls_ids, confs, xywhr)):
            if cf < CONF_MIN: continue
            cx, cy, w, h, th = map(float, obb)
            corners = obb_corners_from_xywhr(cx, cy, w, h, th)
            
            # [수정됨] Global Coordinates (전체 프레임 기준 좌표 계산)
            gx = cx + offset_x
            gy = cy + offset_y

            item_data = {
                'obb': (cx, cy, w, h, th), # 로컬 좌표 (이미지 크롭용)
                'corners': corners,        # 로컬 코너 (드로잉용)
                'cx': cx, 'cy': cy,        # 로컬 중심 (매칭 거리 계산용)
                'gx': gx, 'gy': gy,        # [추가] 글로벌 중심 (CSV/로봇용)
                'conf': float(cf)
            }

            if c == RIGHT_ID:
                crop = obb_warped_crop(pil_img, corners) if USE_PERSPECTIVE_CROP else obb_rotated_crop(pil_img, cx, cy, w, h, th)
                if crop:
                    if ORIENT_NORMALIZE: crop = crop.transpose(Image.FLIP_LEFT_RIGHT)
                    item_data['crop'] = crop
                    item_data['id'] = k
                    item_data['hole'] = None 
                    shoes_r.append(item_data)

            elif c == LEFT_ID:
                crop = obb_warped_crop(pil_img, corners) if USE_PERSPECTIVE_CROP else obb_rotated_crop(pil_img, cx, cy, w, h, th)
                if crop:
                    item_data['crop'] = crop
                    item_data['id'] = k
                    item_data['hole'] = None 
                    shoes_l.append(item_data)
            
            elif c == HOLE_ID: 
                holes.append(item_data)
        
        # [Step 1] Hole matching
        all_shoes = shoes_r + shoes_l
        assign_holes_to_shoes(all_shoes, holes)

        # [Step 2] Shoe pair matching
        has_pair = (len(shoes_r) > 0 and len(shoes_l) > 0)
        pairs = []

        if has_pair:
            crops_r = [s['crop'] for s in shoes_r]
            crops_l = [s['crop'] for s in shoes_l]
            
            R_feats = get_feature_batch(crops_r, resnet, transform, DEVICE)
            L_feats = get_feature_batch(crops_l, resnet, transform, DEVICE)
            S_mat = cosine_sim_matrix(R_feats, L_feats)
            
            match_indices = robust_match(S_mat, use_hungarian=USE_HUNGARIAN)
            
            for ri, li, sc in match_indices:
                pairs.append((shoes_r[ri], shoes_l[li], sc))
        else:
            print("Skipping shoe matching (Missing Left or Right).")

        # [Step 3] Visualization and CSV Saving
        base = pil_img.copy() # base는 ROI 이미지임 (로컬 좌표로 그림)
        draw = ImageDraw.Draw(base)
        font, font_small = load_font()

        def draw_shoe_and_hole(shoe_data, color_shoe, label_prefix):
            draw_obb(draw, shoe_data['corners'], color_shoe, f"{label_prefix}[{shoe_data['id']}]", shoe_data['conf'], font)
            
            if shoe_data['hole']:
                h_data = shoe_data['hole']
                draw_obb(draw, h_data['corners'], (255, 50, 50), "Hold", h_data['conf'], font)
                # Draw vector line (로컬 좌표끼리 연결)
                draw.line([(shoe_data['cx'], shoe_data['cy']), (h_data['cx'], h_data['cy'])], fill=(255, 255, 255), width=2)

        matched_r_ids = set()
        matched_l_ids = set()

        legend_x, legend_y = 10, 10
        draw.text((legend_x, legend_y), fname, fill=(255,255,255), font=font_small, stroke_width=2, stroke_fill=(0,0,0))

        csv_rows = []

        for s_r, s_l, sc in pairs:
            matched_r_ids.add(s_r['id'])
            matched_l_ids.add(s_l['id'])
            
            draw_shoe_and_hole(s_r, (0, 120, 255), "R") 
            draw_shoe_and_hole(s_l, (0, 200, 0), "L")   
            
            draw.line([(s_r['cx'], s_r['cy']), (s_l['cx'], s_l['cy'])], fill=(255, 255, 0), width=3)
            mid = ((s_r['cx'] + s_l['cx'])//2, (s_r['cy'] + s_l['cy'])//2)
            draw.text(mid, f"{sc:.2f}", fill=(255,255,0), font=font_small)

            # [Calculations] Get fx, fz, vec for Right (Global 좌표 기준)
            r_fx, r_fz, r_vec = calculate_shoe_vector(s_r)
            # [Calculations] Get fx, fz, vec for Left (Global 좌표 기준)
            l_fx, l_fz, l_vec = calculate_shoe_vector(s_l)

            # OBB props for logging
            _, _, rw, rh, rr = s_r['obb']
            _, _, lw, lh, lr = s_l['obb']

            csv_rows.append((
                fname,
                round(sc, 4), # Similarity
                # Right Shoe Data
                s_r['id'], round(s_r['conf'], 3), 
                round(r_fx, 1), round(r_fz, 1), round(r_vec, 4), # Global
                # Left Shoe Data
                s_l['id'], round(s_l['conf'], 3), 
                round(l_fx, 1), round(l_fz, 1), round(l_vec, 4), # Global
                # Extra dimensions
                round(rw,1), round(rh,1), round(rr,3),
                round(lw,1), round(lh,1), round(lr,3)
            ))

        for s in shoes_r:
            if s['id'] not in matched_r_ids:
                draw_shoe_and_hole(s, (100, 100, 255), "R(NoPair)")
        for s in shoes_l:
            if s['id'] not in matched_l_ids:
                draw_shoe_and_hole(s, (100, 255, 100), "L(NoPair)")

        assigned_hole_ids = []
        for s in all_shoes:
            if s['hole']: assigned_hole_ids.append(id(s['hole'])) 
            
        for h in holes:
            if id(h) not in assigned_hole_ids:
                draw_obb(draw, h['corners'], (150, 150, 150), "Hold(Unused)", h['conf'], font)

        out_path = os.path.join(OUT_VIZ_DIR, f"{fname}_annotated.jpg")
        base.save(out_path)
        print(f"Saved visualization to {out_path}")

        result_cv = cv2.cvtColor(np.array(base), cv2.COLOR_RGB2BGR)
        cv2.imshow('Match Result', result_cv)

        file_exists = os.path.isfile(OUT_CSV)
        with open(OUT_CSV, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(csv_header)
            writer.writerows(csv_rows)
        print(f"Results saved to {OUT_CSV}")

        num += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()