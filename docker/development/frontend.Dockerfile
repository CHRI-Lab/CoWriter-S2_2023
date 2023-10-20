FROM node

WORKDIR /usr/src/app

COPY ./src/frontend/package*.json .

RUN npm install

COPY ./src/frontend .

RUN npm run build
