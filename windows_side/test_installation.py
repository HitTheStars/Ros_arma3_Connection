"""
测试 Windows 端依赖安装
验证 OpenCV, NumPy, ONNX Runtime DirectML 是否正确安装
"""

import sys

def test_opencv():
    """测试 OpenCV"""
    try:
        import cv2
        print(f"✓ OpenCV 版本: {cv2.__version__}")
        return True
    except ImportError as e:
        print(f"✗ OpenCV 未安装: {e}")
        return False

def test_numpy():
    """测试 NumPy"""
    try:
        import numpy as np
        print(f"✓ NumPy 版本: {np.__version__}")
        return True
    except ImportError as e:
        print(f"✗ NumPy 未安装: {e}")
        return False

def test_onnxruntime():
    """测试 ONNX Runtime"""
    try:
        import onnxruntime as ort
        print(f"✓ ONNX Runtime 版本: {ort.__version__}")
        
        # 检查可用的执行提供器
        providers = ort.get_available_providers()
        print(f"  可用的执行提供器: {providers}")
        
        # 检查 DirectML
        if 'DmlExecutionProvider' in providers:
            print(f"  ✓ DirectML 可用（GPU 加速）")
            return True
        else:
            print(f"  ✗ DirectML 不可用（将使用 CPU）")
            print(f"  提示: 请确保安装了 onnxruntime-directml")
            return False
    
    except ImportError as e:
        print(f"✗ ONNX Runtime 未安装: {e}")
        return False

def test_camera():
    """测试摄像头"""
    try:
        import cv2
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print(f"✗ 无法打开摄像头 0")
            return False
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            print(f"✗ 无法从摄像头读取图像")
            return False
        
        print(f"✓ 摄像头可用，分辨率: {frame.shape[1]}x{frame.shape[0]}")
        return True
    
    except Exception as e:
        print(f"✗ 摄像头测试失败: {e}")
        return False

def main():
    """主函数"""
    print("=" * 50)
    print("  Windows 端依赖安装测试")
    print("=" * 50)
    print()
    
    results = []
    
    print("[1/4] 测试 OpenCV...")
    results.append(test_opencv())
    print()
    
    print("[2/4] 测试 NumPy...")
    results.append(test_numpy())
    print()
    
    print("[3/4] 测试 ONNX Runtime...")
    results.append(test_onnxruntime())
    print()
    
    print("[4/4] 测试摄像头...")
    results.append(test_camera())
    print()
    
    print("=" * 50)
    if all(results):
        print("✓ 所有测试通过！系统已准备就绪。")
        return 0
    else:
        print("✗ 部分测试失败，请检查上述错误信息。")
        return 1

if __name__ == '__main__':
    sys.exit(main())
