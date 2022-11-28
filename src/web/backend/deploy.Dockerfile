FROM python:3


COPY . /workspace/src
RUN pip install --no-cache-dir -r /workspace/src/requirements.txt

ENTRYPOINT ["python", "/workspace/src/main.py" ]
EXPOSE 8000
CMD tail -f /dev/null