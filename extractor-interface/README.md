# Use the docker container

```
sudo docker build --tag=haros .
```

Run the docker container
```
sudo docker run -p 4000:4000 -ti haros:latest
```

Open on your browser the page: http://localhost:4000/ where the Git repository, node and package names can be set
