CUBECLT 1.17.0 으로 버전 변경

1. 이전 빌드 삭제
    -> rmdir .\build

2. CMake 캐시 파일 삭제
    -> rm .\CMakeCache.txt

3. CMake 재구성
    -> cmake -S . -B build