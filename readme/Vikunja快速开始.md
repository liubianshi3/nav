# Vikunja 快速开始

## 1. 准备目录

```bash
mkdir -p ~/vikunja
cd ~/vikunja
mkdir -p db files
```

## 2. 放入 compose 文件

把本目录下的 `docker-compose.vikunja.yml` 复制到 `~/vikunja/docker-compose.yml`。

```bash
cp /home/dell/a2_system_ws/readme/docker-compose.vikunja.yml ~/vikunja/docker-compose.yml
```

## 3. 启动

```bash
cd ~/vikunja
docker compose up -d
```

## 4. 打开网页

浏览器访问：

```text
http://localhost:3456
```

## 5. 常用命令

### 查看状态

```bash
docker compose ps
```

### 查看日志

```bash
docker compose logs -f
```

### 停止

```bash
docker compose down
```

### 重启

```bash
docker compose restart
```

### 升级

```bash
docker compose pull
docker compose up -d
```

## 6. 备份

```bash
tar -czf vikunja-backup-$(date +%F).tar.gz db files docker-compose.yml
```

## 7. 恢复

把备份解压回原目录后执行：

```bash
docker compose up -d
```
