If you need to run docker container in remote hosts , to have access directly to your pc

1. you need to be on the same network , or use vpn (by default the udp communication is blocked so ros2 messages also blocked we will setup rosbridge here)
2. setup env

```bash
mv example.env .env
```

setup the `ROS_DOMAIN_ID` 3. run the docker compose rosbridge

```bash
docker compose -f docker-compose.remote_server.rosbridge.yml -d
```
