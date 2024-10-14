# mono_depth
Monocular Depth Estimation Package \
Tested on Ubuntu 22.04,ros2 humble.
# Installation
## Submodule clone
```bash
cd mono_depth
git submodule update --init --recursive
```
## Dependency Installation
```bash
cd mono_depth/DepthAnythingV2/
pip3 install -r requirements.txt
```
## Model Download and Deployment
Download models from the following URL and place them in mono_depth/models \
※To download all models, run the following shell script
| Model | Params | Checkpoint |
|:-|-:|:-:|
| Depth-Anything-V2-Small | 24.8M | [Download](https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth?download=true) |
| Depth-Anything-V2-Base | 97.5M | [Download](https://huggingface.co/depth-anything/Depth-Anything-V2-Base/resolve/main/depth_anything_v2_vitb.pth?download=true) |
| Depth-Anything-V2-Large | 335.3M | [Download](https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth?download=true) |
| Depth-Anything-V2-Giant | 1.3B | Coming soon |

### Download all models
```bash
cd mono_depth
sh download_models.sh
```

