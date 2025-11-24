import torch
import clip
import numpy as np
from PIL import Image

#  加载CLIP模型
#  Loading CLIP
def load_clip_model(device='cpu'):
    model, preprocess = clip.load("ViT-B/32", device=device)
    return model, preprocess

#  这里把每个tile都resize成了224*224给CLIP计算
#  Send image to CLIP
def compute_tile_embeddings(tiles, model, preprocess, device):
    processed = torch.stack([
        preprocess(Image.fromarray(tile).resize((224, 224))) for tile in tiles
    ]).to(device)
    with torch.no_grad():
        image_features = model.encode_image(processed)
        image_features /= image_features.norm(dim=-1, keepdim=True)
    return image_features.cpu().numpy()

#  计算每个tile与目标prompt的语义的相似度
#  Calculate semantic similarity
def compute_prompt_scores(tile_embeddings, model, device, text_prompts, image_embedding=None):
    if image_embedding is not None:
        return np.dot(tile_embeddings, image_embedding.T).flatten()
    else:
        text_tokens = clip.tokenize(text_prompts).to(device)
        with torch.no_grad():
            text_features = model.encode_text(text_tokens)
            text_features /= text_features.norm(dim=-1, keepdim=True)
        return np.dot(tile_embeddings, text_features.cpu().numpy().T).flatten()
