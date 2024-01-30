# 2024-3dv-advanced-study
Ai Robotics Korea's 2024  3D vision study group's advanced project

---

## Camera Calibration

다양한 카메라 모델들을 기준으로 geometric calibration의 과정에 대해 이해한다.

작성자: 최준혁

이메일: dkwnsgur12@gmail.com

### Dataset path

[Google Drive](https://drive.google.com/file/d/1EOksBolgSh3HrfM2vFcT6uha4GnnK2L9/view?usp=sharing)

데이터셋에는 T265 카메라의 왼쪽 이미지 프레임들과 kalibr 로 보정된 결과 파일이 있다.

### How to Run

#### 1. 데이터셋 다운로드

위 [링크]()를 통해 필요한 데이터셋을 다운로드 받은 후 data 폴더 안에서 압축 해제한다.

#### 2. 실행파일 빌드

```bash
mkdir build
cd build
cmake ..
make

./calibration /path/to/config.json
./undistort /path/to/image /path/to/calib_result.json
```

test.json 파일 내부

- `data_path`: 데이터셋 폴더 경로
- `board_patter`: 체스보드 패턴
- `board_size`: 체스보드 가로세로 사이즈(단위: meter)
- `camera_model`: 사용할 카메라 모델.
  - `SingleSphere`: `UCM` 카메라 모델, 사용 파라미터 (`alpha`)
  - `DoubleSphere`: `Double Sphere Camera Model` 카메라 모델, 사용 파라미터 (`alpha`, `lambda`)
  - `TripleSphere`: `Triple Sphere Camera Model` 카메라 모델, 사용 파라미터 (`alpha`, `lambda`, `gamma`)
  - `BrownConrady`: `BrownConrady` 카메라 모델, 사용 파라미터 (`k1`, `k2`)
  - `KannalaBrandt`: `KannalaBrandt` 카메라 모델, 사용 파라미터 (`k1`, `k2`, `k3`, `k4`)
- `output_path`: 보정 결과 경로
- `output_name`: 보정 결과 파일 이름

```json
{
    "data_path": "/path/to/dataset",    
    "board_pattern": [6, 9],
    "board_size": [0.05, 0.05],
    // "camera_model": "SingleSphere",
    "camera_model": "DoubleSphere",
    // "camera_model": "TripleSphere",
    // "camera_model": "BrownConrady",
    // "camera_model": "KannalaBrandt",
    "output_path": "/path/to/output",
    "output_name": "result.json"
}
```

#### 3. 결과 파일 해석

result.json 파일 내부

- `avg_residual`: 카메라 보정의 mean residual 값, 단위: pixel
- `camera_model`: 사용된 카메라 모델
- `number_of_used_images`: 이전 보정 과정 중 사용된 이미지 수
- `parameters`: 카메라 파라미터. fx, fy, cx, cy, ... 순서

```json
{
    "avg_residual" : 0.070837114453125141,
    "camera_model" : "DoubleSphere",
    "number_of_used_images" : 397,
    "parameters" : 
    [
        285.04119723202768,
        285.25044458408809,
        426.52144861596793,
        386.41686040643305,
        0.00094578527212422743,
        0.64125887224024325
    ]
}
```

### Reference

- https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- https://inria.hal.science/inria-00267247/document
- https://arxiv.org/pdf/1807.08957
- https://ieeexplore.ieee.org/document/1642666
- https://openaccess.thecvf.com/content/CVPR2023/papers/Xie_OmniVidar_Omnidirectional_Depth_Estimation_From_Multi-Fisheye_Images_CVPR_2023_paper.pdf