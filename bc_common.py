from torch.utils.data import Dataset
import glob
import os
import torch
import numpy as np
from torchvision.io import read_image
import torch.nn as nn
import torch.nn.functional as F


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 6, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(6, 16, 5)
        self.fc1 = nn.Linear(11664, 512)
        self.fc2 = nn.Linear(512, 128)
        self.fc3 = nn.Linear(128, 3)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = torch.flatten(x, 1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x


class CustomImageDataset(Dataset):
    def __init__(self, dataset_type='train', max_images=None):
        self.dataset_type = dataset_type
        self.n_images = len(glob.glob(f'dataset/{self.dataset_type}/*.jpg'))
        if max_images:
            self.n_images = min(self.n_images, max_images)

    def __len__(self):
        return self.n_images

    def __getitem__(self, idx):
        image_path = os.path.join('dataset', self.dataset_type, f'{idx}.jpg')
        label_path = os.path.join('dataset', self.dataset_type, f'{idx}.csv')
        image = read_image(image_path).to(torch.float32) / 255
        label = torch.from_numpy(np.loadtxt(label_path, delimiter=',')).to(torch.float32)
        return image, label
