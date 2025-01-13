


# Trouble shooting

## xhost 권한 설정이 올바르게 적용되었는지 확인:

```base
xhost +local:docker
```

## DISPLAY 환경 변수가 올바르게 설정되었는지 확인:

```base
export DISPLAY=:0
```



```
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
    command: >
      bash -c "
      cd /workspace/simulation_code &&
      .python my_application.py
      "

```