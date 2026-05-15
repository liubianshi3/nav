# Vikunja Docker Compose 安装与维护说明

本文用于说明如何在本机通过 Docker Compose 安装、启动、停止和维护 Vikunja 任务管理服务，方便后续长期使用和迁移。

## 1. 目标

Vikunja 用于提供一个轻量的任务清单/看板界面，适合记录：

- 当前要做什么
- 正在做什么
- 已完成什么
- 待排查的问题
- 后续跟进事项

推荐用 Docker Compose 部署，原因是：

- 启动方式统一
- 后续备份和迁移简单
- 方便升级版本
- 比单条 `docker run` 更易维护

## 2. 目录结构建议

建议单独放在一个独立目录中，例如：

```bash
~/vikunja
```

建议目录结构如下：

```text
~/vikunja/
├── docker-compose.yml
├── db/
├── files/
└── README.md
```

其中：

- `db/`：存放数据库文件
- `files/`：存放附件、导出文件等
- `docker-compose.yml`：容器启动配置

## 3. 安装前准备

### 3.1 安装 Docker

确认本机已安装并启动 Docker：

```bash
systemctl status docker
```

如果没有安装 Docker，请先安装 Docker Engine 和 Docker Compose 插件。

### 3.2 创建工作目录

```bash
mkdir -p ~/vikunja
cd ~/vikunja
mkdir -p db files
```

## 4. Docker Compose 配置

创建 `docker-compose.yml`，内容如下：

```yaml
services:
  vikunja:
    image: vikunja/vikunja:latest
    container_name: vikunja
    restart: unless-stopped
    ports:
      - "3456:3456"
    environment:
      VIKUNJA_SERVICE_PUBLICURL: http://localhost:3456
      VIKUNJA_DATABASE_TYPE: sqlite
      VIKUNJA_DATABASE_PATH: /db/vikunja.db
    volumes:
      - ./db:/db
      - ./files:/app/vikunja/files
```

## 5. 启动与访问

启动服务：

```bash
docker compose up -d
```

查看状态：

```bash
docker compose ps
```

查看日志：

```bash
docker compose logs -f
```

浏览器访问：

```text
http://localhost:3456
```

## 6. 日常维护命令

### 启动

```bash
docker compose up -d
```

### 停止

```bash
docker compose down
```

### 重启

```bash
docker compose restart
```

### 升级镜像

```bash
docker compose pull
docker compose up -d
```

### 查看运行状态

```bash
docker compose ps
```

### 查看日志

```bash
docker compose logs -f
```

## 7. 数据备份

Vikunja 使用 SQLite 时，核心数据主要在 `db/` 目录中。

建议定期备份以下目录：

- `db/`
- `files/`

示例备份命令：

```bash
tar -czf vikunja-backup-$(date +%F).tar.gz db files docker-compose.yml
```

恢复时，把备份解压回原目录，再执行：

```bash
docker compose up -d
```

## 8. 常见问题

### 8.1 页面打不开

先检查容器是否在运行：

```bash
docker compose ps
```

再检查端口是否被占用：

```bash
ss -ltnp | grep 3456
```

### 8.2 数据丢失

通常是因为：

- 没有挂载 `db/`
- 容器被删除后数据没有持久化
- 备份没有定期执行

### 8.3 想迁移到其他机器

直接把以下内容一起迁走即可：

- `docker-compose.yml`
- `db/`
- `files/`

在新机器上执行 `docker compose up -d` 即可恢复。

## 9. 推荐使用方式

如果只是个人使用，建议一直保持 SQLite + Docker Compose 这套最简配置。

如果后面需要多人协作或更高可靠性，再考虑升级到：

- PostgreSQL
- 反向代理
- 域名/HTTPS
- 自动备份脚本

## 10. 当前建议结论

对于本机日常任务清单管理，Docker Compose + SQLite 是最轻量、最容易维护的方案。

如果后续只是想先快速使用，可以直接按本文的最简方案部署；如果后面想长期交付或迁移，再扩展数据库和备份策略。
