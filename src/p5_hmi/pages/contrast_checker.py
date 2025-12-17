#Define two colors rgba kivy style normalized between 0-1
buttons1 = [
    [0.23, 0.63, 0.92, 1],
    [0.11, 0.74, 0.61, 1],
    [0.24, 0.80, 0.45, 1],
    [0.54, 0.46, 0.98, 1],
    [0.98, 0.78, 0.29, 1],
    [0.94, 0.36, 0.36, 1],
    [0.86, 0.52, 0.60, 1],
    [0.62, 0.65, 0.78, 1]
]

buttons2 = [
    [0.168, 0.46, 0.672, 1.0],
    [0.074, 0.496, 0.409, 1.0],
    [0.149, 0.496, 0.279, 1.0],
    [0.443, 0.377, 0.804, 1.0],
    [0.529, 0.421, 0.157, 1.0],
    [0.743, 0.284, 0.284, 1.0],
    [0.611, 0.369, 0.426, 1.0],
    [0.415, 0.436, 0.523, 1.0]
]

bg_primary = (0.11, 0.15, 0.25, 1)
bg_secondary = (0.18, 0.24, 0.35, 1)
btns = [
    (0.256, 0.624, 0.440, 1), #success green
    (0.773, 0.496, 0.187, 1), #accent_orange
    (0.896, 0.411, 0.293, 1), #accent_coral
    (0.540, 0.460, 0.980, 1), #browse_btn
    (0.515, 0.561, 0.636, 1), #btn neutral
    (0.408, 0.445, 0.504, 1), #btn dark neutral
    (0.496, 0.353, 0.688, 1) #delay green
]

text = [0.96, 0.96, 0.98, 1]
text_muted = (0.75, 0.78, 0.82, 1)
dark_text = (0.247, 0.247, 0.247, 1)

#Calculate contrast ratio
def luminance(color):
    r, g, b, a = color
    r = r / 12.92 if r <= 0.03928 else ((r + 0.055) / 1.055) ** 2.4
    g = g / 12.92 if g <= 0.03928 else ((g + 0.055) / 1.055) ** 2.4
    b = b / 12.92 if b <= 0.03928 else ((b + 0.055) / 1.055) ** 2.4
    return 0.2126 * r + 0.7152 * g + 0.0722 * b
def contrast_ratio(color1, color2):
    L1 = luminance(color1)
    L2 = luminance(color2)
    if L1 > L2:
        return (L1 + 0.05) / (L2 + 0.05)
    else:
        return (L2 + 0.05) / (L1 + 0.05)
# for i, button in enumerate(buttons2):
#     ratio = contrast_ratio(button, text)
#     print(f"Contrast ratio between button {i+1} and text: {ratio:.2f}:1")

for i, button in enumerate(btns):
    ratio = contrast_ratio(button, text)
    print(f"Contrast ratio between button {i+1} and text: {ratio:.2f}:1")