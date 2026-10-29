FROM python:3.11-slim
WORKDIR /app
COPY pyproject.toml README.md ./
RUN pip install --no-cache-dir 'numpy>=2,<3' 'scipy>=1.14,<2' \
    'opencv-python-headless>=4.10,<5' 'zerorpc==0.6.3'
COPY src ./src
RUN pip install --no-cache-dir --no-deps .
COPY configs ./configs
ENTRYPOINT ["python", "-m", "auto_reset"]
