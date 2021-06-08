FROM python:3.9

ENV DASH_DEBUG_MODE True
COPY src/app /app
WORKDIR /app
RUN set -ex && \
    pip3 install -r requirements.txt
EXPOSE 8050
CMD ["python", "app.py"]
