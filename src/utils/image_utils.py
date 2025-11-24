import cv2

def split_image_into_tiles(image_rgb, grid):
    h, w, _ = image_rgb.shape
    rows, cols = grid
    tile_h = h // rows
    tile_w = w // cols
    tiles = []
    for i in range(rows):
        for j in range(cols):
            y1 = i * tile_h
            y2 = (i+1)*tile_h if i != rows-1 else h
            x1 = j * tile_w
            x2 = (j+1)*tile_w if j != cols-1 else w
            tile = image_rgb[y1:y2, x1:x2]
            tiles.append(tile)
    return tiles

def split_image_into_tiles_grid(image_rgb, grid):
    resized = cv2.resize(image_rgb, (640, 480))  # 也可以传入 self.target_size
    return split_image_into_tiles(resized, grid)

def draw_scores(frame_bgr, scores, grid=(3, 3), highlight_idx=None, show=False):
    h, w, _ = frame_bgr.shape
    rows, cols = grid
    tile_h = h // rows
    tile_w = w // cols

    for i in range(rows):
        for j in range(cols):
            idx = i * cols + j
            x1, y1 = j * tile_w, i * tile_h
            x2, y2 = (j + 1) * tile_w, (i + 1) * tile_h
            score_label = f"{scores[idx]:.2f}"
            color = (0, 255, 0) if highlight_idx == idx else (0, 0, 255)
            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame_bgr, score_label, (x1 + 5, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    if show:
        cv2.imshow("Tile Score Visualization", frame_bgr)
        cv2.waitKey(1)
