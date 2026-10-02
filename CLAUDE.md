# arm-lab 작업 규칙

- 기능·화면·설치 방법·파일 이름이 바뀌면 **같은 커밋에서** `README.md`(사용법)를 갱신한다.
  구조·설계 이유가 바뀌면 `docs/DEVELOPMENT.md` 도 갱신한다.
- 화면이 바뀌었으면 `docs/img/` 의 해당 캡처를 다시 찍는다 (README 에 쓰인 이미지).
- 실행 파일은 `main.py`, 보조 모듈은 `armlab_*.py`, 도구는 `tools_*.py`. 런타임 파일은 `armlab_*` 이름으로 레포 폴더(또는 `ARMLAB_HOME`)에 생긴다.
- 화면 문자열을 바꾸면 영어 사전(`armlab_i18n_en.json`)에도 반영한다.
