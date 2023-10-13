FROM node as development

WORKDIR /usr/src/app

COPY ./src/frontend/package*.json .

RUN npm install

COPY ./src/frontend .

RUN npm run build

FROM nginx:stable-alpine as production

COPY --from=development /usr/src/app/dist /usr/share/nginx/html

COPY ./src/frontend/nginx.conf /etc/nginx/conf.d/default.conf

CMD ["nginx", "-g", "daemon off;"]
