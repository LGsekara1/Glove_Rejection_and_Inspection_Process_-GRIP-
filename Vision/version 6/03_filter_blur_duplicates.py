import cv2
import shutil
from pathlib import Path
from PIL import Image
import imagehash

IN_DIR = Path("data/extracted_frames")
GOOD_DIR = Path("data/filtered_frames/good")
BLUR_DIR = Path("data/filtered_frames/blurry")
DUP_DIR = Path("data/filtered_frames/duplicates")

for d in [GOOD_DIR, BLUR_DIR, DUP_DIR]:
    d.mkdir(parents=True, exist_ok=True)

BLUR_THRESHOLD = 80.0
HASH_DISTANCE = 4
seen_hashes = []

def blur_score(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()

def is_duplicate(path):
    img_hash = imagehash.phash(Image.open(path))
    for h in seen_hashes:
        if abs(img_hash - h) <= HASH_DISTANCE:
            return True
    seen_hashes.append(img_hash)
    return False

for img_path in IN_DIR.rglob("*.jpg"):
    img = cv2.imread(str(img_path))
    if img is None:
        continue

    score = blur_score(img)
    if score < BLUR_THRESHOLD:
        shutil.copy(str(img_path), str(BLUR_DIR / img_path.name))
        continue

    if is_duplicate(img_path):
        shutil.copy(str(img_path), str(DUP_DIR / img_path.name))
        continue

    shutil.copy(str(img_path), str(GOOD_DIR / img_path.name))

print("Done filtering. Manually review kept/rejected frames.")
