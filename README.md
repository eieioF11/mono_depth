# mono_depth
単眼深度推定パッケージ
## 環境構築
### サブモジュールクローン
```bash
cd mono_depth
git submodule update --init --recursive
```
### 依存関係インストール
```bash
cd mono_depth/DepthAnythingV2/
pip3 install -r requirements.txt
```
### モデルのダウンロードと配置
以下のURLからモデルをダウンロードしmono_depth/modelsに配置
| Model | Params | Checkpoint |
|:-|-:|:-:|
| Depth-Anything-V2-Small | 24.8M | [Download](https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth?download=true) |
| Depth-Anything-V2-Base | 97.5M | [Download](https://huggingface.co/depth-anything/Depth-Anything-V2-Base/resolve/main/depth_anything_v2_vitb.pth?download=true) |
| Depth-Anything-V2-Large | 335.3M | [Download](https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth?download=true) |
| Depth-Anything-V2-Giant | 1.3B | Coming soon |


