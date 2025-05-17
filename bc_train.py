import torch
import torch.optim as optim
import torch.nn as nn
from bc_common import CustomImageDataset, Net
from tqdm import tqdm

VALIDATION_METRIC = 'loss'

def evaluate_loss(net, device, dataset, criterion):
    running_loss = 0
    net.eval()
    for i, data in enumerate(dataset, 0):
        inputs, labels = data
        inputs = inputs.to(device)
        labels = labels.to(device)
        outputs = net(inputs)
        loss = criterion(outputs, labels)
        running_loss += loss.item() * inputs.size(0)

    average_loss = running_loss / len(dataset.dataset)
    print(f'Average validation loss: {average_loss}')
    return average_loss


def main():
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    print('device:', device)
    # Split dataset into train and test
    train_dataset = torch.utils.data.DataLoader(
        CustomImageDataset(dataset_type='train'), batch_size=64, shuffle=True, num_workers=25)
    val_dataset = torch.utils.data.DataLoader(
        CustomImageDataset(dataset_type='val'), batch_size=64, shuffle=True, num_workers=25)
    print('data loaded')

    # Validation
    best_score = None

    # Train
    net = Net().to(device)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(net.parameters(), lr=0.001)
    for epoch in tqdm(range(100)):
        running_loss = 0.0
        net.train()
        for i, data in enumerate(train_dataset, 0):
            # get the inputs; data is a list of [inputs, labels]
            inputs, labels = data
            inputs = inputs.to(device)
            labels = labels.to(device)

            # zero the parameter gradients
            optimizer.zero_grad()

            # forward + backward + optimize
            outputs = net(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            # print statistics
            running_loss += loss.item()
            if i % 1000 == 999:
                print(f'[{epoch + 1}, {i + 1:5d}] loss: {running_loss / 1000:.3f}')
                running_loss = 0.0
                #torch.save(net.state_dict(), './bc_model_temp.pth')

        # Evaluate
        loss = evaluate_loss(net, device, val_dataset, criterion)
        if best_score is None or abs(loss) < abs(best_score):
            best_score = loss
            torch.save(net.state_dict(), './bc_model_best.pth')
            print('Best model saved based on loss')

    # torch.save(net.state_dict(), './bc_model_last.pth')
    print('Finished Training')


if __name__ == '__main__':
    main()
