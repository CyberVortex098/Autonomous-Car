# Running the files

## Debugger

Install libraries using
```sh
pip install PyQt5 pyserial matplotlib
```

## Monitor

<ol>
  <li>make it executable using <code>chmod +x monitor.sh</code></li>
  <li>Run <code>sudo ./monitor.sh</code></li>
</ol>

## Camera

```sh
ffplay -f v4l2 -video_size 640x480 -framerate 30 -i /dev/video0
```
