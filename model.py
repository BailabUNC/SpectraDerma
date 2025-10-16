import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
import matplotlib.pyplot as plt

# Import tqdm with Jupyter notebook support
try:
    from tqdm.notebook import tqdm
except ImportError:
    from tqdm import tqdm


class SpectraDermaPPGDataset(Dataset):
    """Dataset class for PPG segments with metadata, DC time series, and binary labels."""
    
    def __init__(self, segments_path, labels):
        """
        Initialize PPG dataset.
        
        Parameters:
        - segments_path: path to .pt file with dict containing 'segments', 'metadata', 'dc_values'
        - labels: binary labels (0=normal, 1=alcohol) of length N
        """
        data_dict = torch.load(segments_path)
        
        # Extract components from the dictionary
        self.segments = data_dict['segments']  # Shape: (N, T, C)
        self.metadata = data_dict.get('metadata', None)  # Shape: (N, C, 3) or None
        dc_values_raw = data_dict.get('dc_values', None)  # Shape: (N, C)
        if dc_values_raw is not None:
            # Broadcast (N, C) to (N, T, C) where T matches segments time dimension
            T = self.segments.shape[1]  # Get time dimension from segments
            self.dc_values = dc_values_raw.unsqueeze(1).expand(-1, T, -1)  # Shape: (N, T, C)
        else:
            self.dc_values = None
        self.labels = torch.tensor(labels, dtype=torch.long)
        
        assert len(self.segments) == len(self.labels), "Segments and labels must have same length"
        
        # Validate metadata and DC values if they exist
        if self.metadata is not None:
            assert len(self.metadata) == len(self.labels), "Metadata and labels must have same length"
        if self.dc_values is not None:
            assert len(self.dc_values) == len(self.labels), "DC values and labels must have same length"
        
    def __len__(self):
        return len(self.segments)
    
    def __getitem__(self, idx):
        item = {'segments': self.segments[idx], 'labels': self.labels[idx]}
        
        if self.metadata is not None:
            item['metadata'] = self.metadata[idx]
        if self.dc_values is not None:
            item['dc_values'] = self.dc_values[idx]
            
        return item


class SpectraDermaPPGClassifier(nn.Module):
    """GRU-based classifier for PPG segments with metadata and DC value fusion."""
    
    def __init__(self, input_size=6, hidden_size=64, num_layers=2, dropout=0.2, 
                 meta_hidden=32):
        """
        Initialize PPG classifier.
        
        Parameters:
        - input_size: number of PPG channels (default: 6)
        - hidden_size: GRU hidden dimension (default: 64)
        - num_layers: number of GRU layers (default: 2)
        - dropout: dropout rate (default: 0.2)
        - meta_hidden: hidden dimension for metadata processing (default: 32)
        """
        super(SpectraDermaPPGClassifier, self).__init__()
        
        # GRU for combined PPG segments and DC time series
        # Input will be stacked: [PPG, DC] -> shape (B, 256, 12)
        combined_input_size = input_size * 2  # PPG (6) + DC (6) = 12
        self.gru = nn.GRU(
            combined_input_size, hidden_size, num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0
        )
        
        # Metadata processing: (batch, 3, 6) -> (batch, 18) -> (batch, meta_hidden//2)
        self.meta_fc = nn.Sequential(
            nn.Linear(18, meta_hidden),  # 3 * 6 = 18 features
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(meta_hidden, meta_hidden // 2)
        )
        
        # Final classifier
        total_features = hidden_size + (meta_hidden // 2)  # 64 + 16 = 80
        self.dropout = nn.Dropout(dropout)
        self.classifier = nn.Sequential(
            nn.Linear(total_features, total_features // 2),  # 80 -> 40
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(total_features // 2, 2)  # 40 -> 2 (binary classification)
        )
        
    def forward(self, segments, metadata, dc_values):
        """
        Forward pass with segments, metadata, and DC values.
        
        Parameters:
        - segments: PPG segments (batch, time_steps, channels)
        - metadata: segment metadata (batch, 3, 6) 
        - dc_values: DC time series (batch, time_steps, channels) - same shape as segments
        
        Returns:
        - logits: classification logits (batch, 2)
        """
        # Stack PPG segments and DC time series along channel dimension
        # segments: (B, 256, 6), dc_values: (B, 256, 6) -> combined: (B, 256, 12)
        combined_input = torch.cat([segments, dc_values], dim=2)
        
        # Process combined input through GRU
        gru_out, _ = self.gru(combined_input)
        gru_features = self.dropout(gru_out[:, -1, :])  # Use last timestep output
        
        # Process metadata: flatten (batch, 3, 6) -> (batch, 18)
        meta_flat = metadata.view(metadata.size(0), -1)
        meta_features = self.meta_fc(meta_flat)
        
        # Combine GRU features and metadata features
        combined_features = torch.cat([gru_features, meta_features], dim=1)
        
        # Final classification
        return self.classifier(combined_features)


def train_model(model, train_loader, val_loader, num_epochs=50, learning_rate=0.001, weight_decay=0.0, device='cpu', show_progress=True):
    """
    Train the PPG classifier with progress bars.
    
    Parameters:
    - model: PPGClassifier instance
    - train_loader: training data loader
    - val_loader: validation data loader
    - num_epochs: number of training epochs
    - learning_rate: learning rate for optimizer
    - weight_decay: L2 regularization strength (default: 0.0)
    - device: device to train on ('cpu' or 'cuda')
    - show_progress: whether to show progress bars (default: True)
    
    Returns:
    - train_losses: list of training losses
    - val_accuracies: list of validation accuracies
    - train_accuracies: list of training accuracies
    - val_losses: list of validation losses
    """
    model.to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate, weight_decay=weight_decay)
    
    train_losses = []
    val_accuracies = []
    train_accuracies = []
    val_losses = []
    
    # Create epoch progress bar
    epoch_pbar = tqdm(range(num_epochs), desc='Training', unit='epoch', disable=not show_progress)
    
    for epoch in epoch_pbar:
        # Training phase
        model.train()
        epoch_losses = []
        train_correct = 0
        train_total = 0
        
        # Create batch progress bar for training
        train_pbar = tqdm(train_loader, desc=f'Epoch {epoch+1}/{num_epochs} - Train', 
                         unit='batch', leave=False, disable=not show_progress)
        
        for batch in train_pbar:
            # Extract data from batch dictionary
            segments = batch['segments'].to(device)
            metadata = batch['metadata'].to(device)
            dc_values = batch['dc_values'].to(device)
            labels = batch['labels'].to(device)
            
            optimizer.zero_grad()
            outputs = model(segments, metadata, dc_values)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            epoch_losses.append(loss.item())
            
            # Calculate training accuracy
            predicted = torch.argmax(outputs, dim=1)
            train_correct += (predicted == labels).sum().item()
            train_total += labels.size(0)
            
            # Update training progress bar with current loss and accuracy
            current_train_acc = train_correct / train_total if train_total > 0 else 0
            train_pbar.set_postfix({'loss': f'{loss.item():.4f}', 'acc': f'{current_train_acc:.4f}'})
        
        avg_train_loss = sum(epoch_losses) / len(epoch_losses)
        train_accuracy = train_correct / train_total
        train_losses.append(avg_train_loss)
        train_accuracies.append(train_accuracy)
        
        # Validation phase
        model.eval()
        val_correct = 0
        val_total = 0
        val_epoch_losses = []
        
        # Create batch progress bar for validation
        val_pbar = tqdm(val_loader, desc=f'Epoch {epoch+1}/{num_epochs} - Val', 
                       unit='batch', leave=False, disable=not show_progress)
        
        with torch.no_grad():
            for batch in val_pbar:
                segments = batch['segments'].to(device)
                metadata = batch['metadata'].to(device)
                dc_values = batch['dc_values'].to(device)
                labels = batch['labels'].to(device)
                
                outputs = model(segments, metadata, dc_values)
                val_loss = criterion(outputs, labels)
                val_epoch_losses.append(val_loss.item())
                
                predicted = torch.argmax(outputs, dim=1)
                val_correct += (predicted == labels).sum().item()
                val_total += labels.size(0)
                
                # Update validation progress bar with current accuracy and loss
                current_val_acc = val_correct / val_total if val_total > 0 else 0
                val_pbar.set_postfix({'loss': f'{val_loss.item():.4f}', 'acc': f'{current_val_acc:.4f}'})
        
        val_accuracy = val_correct / val_total
        avg_val_loss = sum(val_epoch_losses) / len(val_epoch_losses)
        val_accuracies.append(val_accuracy)
        val_losses.append(avg_val_loss)
        
        # Update epoch progress bar with metrics
        epoch_pbar.set_postfix({
            'train_loss': f'{avg_train_loss:.4f}',
            'train_acc': f'{train_accuracy:.4f}',
            'val_loss': f'{avg_val_loss:.4f}',
            'val_acc': f'{val_accuracy:.4f}'
        })
        
        # Print detailed info every 10 epochs
        if (epoch + 1) % 10 == 0:
            print(f'Epoch [{epoch+1}/{num_epochs}], Train Loss: {avg_train_loss:.4f}, Train Acc: {train_accuracy:.4f}, Val Loss: {avg_val_loss:.4f}, Val Acc: {val_accuracy:.4f}')
    
    epoch_pbar.close()
    return train_losses, val_accuracies, train_accuracies, val_losses


def evaluate_model(model, test_loader, device='cpu'):
    """
    Evaluate the trained model.
    
    Parameters:
    - model: trained PPGClassifier instance
    - test_loader: test data loader
    - device: device to evaluate on
    
    Returns:
    - accuracy: test accuracy
    - predictions: predicted labels
    - true_labels: true labels
    """
    model.eval()
    predictions, true_labels = [], []
    
    with torch.no_grad():
        for batch in test_loader:
            segments = batch['segments'].to(device)
            metadata = batch['metadata'].to(device)
            dc_values = batch['dc_values'].to(device)
            labels = batch['labels'].to(device)
            
            outputs = model(segments, metadata, dc_values)
            predicted = torch.argmax(outputs, dim=1)
            
            predictions.extend(predicted.cpu())
            true_labels.extend(labels.cpu())
    
    accuracy = accuracy_score(true_labels, predictions)
    
    return accuracy, predictions, true_labels


def plot_training_history(train_losses, val_accuracies, train_accuracies=None, val_losses=None):
    """Plot training and validation loss and accuracy on the same plot with twin y-axes."""
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # Plot losses on left y-axis
    color1 = 'tab:red'
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('Loss', color=color1)
    ax1.plot(train_losses, color=color1, linestyle='-', label='Training Loss', linewidth=2)
    if val_losses is not None:
        ax1.plot(val_losses, color=color1, linestyle='--', label='Validation Loss', linewidth=2)
    ax1.tick_params(axis='y', labelcolor=color1)
    
    # Create twin y-axis for accuracies
    ax2 = ax1.twinx()
    color2 = 'tab:blue'
    ax2.set_ylabel('Accuracy', color=color2)
    ax2.plot(val_accuracies, color=color2, linestyle='--', label='Validation Accuracy', linewidth=2)
    if train_accuracies is not None:
        ax2.plot(train_accuracies, color=color2, linestyle='-', label='Training Accuracy', linewidth=2)
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Add title and legend
    ax1.set_title('Training History: Loss and Accuracy')
    
    # Combine legends from both axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    
    plt.tight_layout()
    plt.show()


# Example usage
if __name__ == "__main__":
    # Example of how to use the model
    print("PPG Binary Classifier Example")
    print("=" * 40)
    
    # Create dummy data for demonstration with new format
    # In practice, you would load real .pt files and labels from SpectraDermaPPGAnalyzer.compile()
    dummy_segments = torch.randn(100, 256, 6)  # 100 segments, 256 time steps, 6 channels
    dummy_metadata = torch.randn(100, 6, 3)  # 100 segments, 6 channels, 3 features [height, interval, width]
    dummy_dc_values = torch.randn(100, 256, 6)  # 100 segments, 256 time steps, 6 channels (time series)
    dummy_labels = torch.randint(0, 2, (100,))  # Random binary labels
    
    # Save dummy data in new format (dict structure)
    dummy_data = {
        'segments': dummy_segments,
        'metadata': dummy_metadata,
        'dc_values': dummy_dc_values
    }
    torch.save(dummy_data, "dummy_segments.pt")
    
    # Create dataset and data loaders
    dataset = SpectraDermaPPGDataset("dummy_segments.pt", dummy_labels)
    
    # Split into train/val/test (70/15/15)
    train_size = int(0.7 * len(dataset))
    val_size = int(0.15 * len(dataset))
    test_size = len(dataset) - train_size - val_size
    
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(
        dataset, [train_size, val_size, test_size]
    )
    
    # Custom collate function to handle the new data format
    def collate_fn(batch):
        """Custom collate function to handle dict-based batch data."""
        if isinstance(batch[0], dict):
            # New format: batch contains dicts
            collated = {}
            for key in batch[0].keys():
                if key in ['segments', 'metadata', 'dc_values', 'labels']:
                    collated[key] = torch.stack([item[key] for item in batch])
            return collated
        else:
            # Old format: batch contains tuples
            return torch.utils.data.dataloader.default_collate(batch)
    
    # Create data loaders
    train_loader = DataLoader(train_dataset, batch_size=16, shuffle=True, collate_fn=collate_fn)
    val_loader = DataLoader(val_dataset, batch_size=16, shuffle=False, collate_fn=collate_fn)
    test_loader = DataLoader(test_dataset, batch_size=16, shuffle=False, collate_fn=collate_fn)
    
    # Initialize model with enhanced architecture
    model = SpectraDermaPPGClassifier(
        input_size=6,
        hidden_size=64,
        num_layers=2,
        dropout=0.2,
        meta_hidden=32   # Hidden dimension for metadata processing
    )
    
    print(f"Model parameters: {sum(p.numel() for p in model.parameters()):,}")
    
    # Train model
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Training on: {device}")
    
    train_losses, val_accuracies = train_model(
        model, train_loader, val_loader, 
        num_epochs=50, learning_rate=0.001, device=device
    )
    
    # Plot training history
    plot_training_history(train_losses, val_accuracies)
    
    # Evaluate model
    accuracy, predictions, true_labels = evaluate_model(model, test_loader, device)
    
    # Save trained model
    torch.save(model.state_dict(), "ppg_classifier.pth")
    print("\nModel saved to 'ppg_classifier.pth'")
