# 터미널에서 같은 경로에 있지 않는 파일 모듈로 쓰기
```python3
# 필요한 파일 경로 C:\Users\User1
# 파일에 이름 file.py 클래스에 이름 Hello
import sys
sys.path.append(C:\Users\User1)

# 아래에 명령어로 제대로 추가되었는지 확인한다.
sys.path

# 사용하는 방법
from file.py import Hello
```
다른 곳에서 모듈로 가져왔을 때는 실행되지 않고 직접 실행시켰을 때만 실행하기
```
if __name__ == '__main__':
    원하는 소스코드
```