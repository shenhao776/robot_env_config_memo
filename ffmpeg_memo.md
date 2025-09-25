# 用cuda转mp4
ffmpeg -hwaccel cuda -i aa.webm -vf scale=1920:1080 -c:v hevc_nvenc -preset medium -cq 25 -c:a aac -b:a 128k output_1080p.mp4
