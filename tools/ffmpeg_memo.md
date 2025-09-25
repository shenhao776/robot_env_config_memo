```bash
# 用cuda转mp4,但是拷贝到 mac 上就播放不了了
ffmpeg -hwaccel cuda -i aa.webm -vf scale=1920:1080 -c:v hevc_nvenc -preset medium -cq 25 -c:a aac -b:a 128k output_1080p.mp4

# mac的命令，实测可用
ffmpeg -i input.webm -c:v libx264 -crf 23 -c:a aac -b:a 128k output.mp4
```
