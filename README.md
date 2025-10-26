# Installation og kørsel af HMI med Kivy/KivyMD

## 1. Opret og aktiver virtuelt miljø

```bash
python3 -m venv kivy_venv
source kivy_venv/bin/activate
```

## 2. Installer Kivy og KivyMD

Kør følgende kommandoer i det aktiverede virtuelle miljø:

```bash
python -m pip install --upgrade pip wheel setuptools
python -m pip install kivy[base] kivymd
```

> **Bemærk:** På nogle systemer kan du have brug for ekstra systempakker (fx `libgl1-mesa-dev`, `gstreamer`, `python3-dev`). Se [Kivys dokumentation](https://kivy.org/doc/stable/gettingstarted/installation.html) hvis du får fejl.

## 3. Byg workspace (ROS2)

```bash
colcon build
```

## 4. Kør HMI GUI

Aktivér det virtuelle miljø og brug startscriptet:

```bash
source kivy_venv/bin/activate
./start_hmi.sh
```

---
Scriptet forudsætter at `kivy_venv` ligger i repo root og at workspace er bygget (`colcon build`).
