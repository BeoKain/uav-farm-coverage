import math

def encode_legacy_gauss(flu_tesla):
 x,y,z=flu_tesla
 if not all(math.isfinite(v) for v in (x,y,z)):raise ValueError('Nonfinite native magnetometer')
 return (y*1e4,-x*1e4,-z*1e4)
