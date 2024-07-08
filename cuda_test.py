import torch

if torch.cuda.is_available():
    device = torch.device("cuda")
    print("CUDA is available")
else:
    device = torch.device("cpu")
    print("CUDA is not available, using CPU instead")

x = torch.randn(3, 3).to(device)   # создаем случайный тензор
y = torch.ones(3, 3).to(device)    # создаем тензор из единиц

z = x + y

z_cpu = z.to("cpu")

print("Result tensor (on CPU):")
print(z_cpu)

