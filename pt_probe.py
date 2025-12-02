import torch
import sys
import os

def probe_pt_file(file_path):
    """
    Probe a .pt file and print information about tensors stored in it.
    
    Args:
        file_path (str): Path to the .pt file
    """
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' does not exist.")
        return
    
    try:
        # Load the .pt file
        data = torch.load(file_path, map_location='cpu')
        
        print(f"=== Probing {file_path} ===")
        print(f"File size: {os.path.getsize(file_path) / (1024*1024):.2f} MB")
        print(f"Data type: {type(data)}")
        
        if isinstance(data, dict):
            print(f"Number of keys: {len(data)}")
            print("\n--- Dictionary Contents ---")
            
            for key, value in data.items():
                print(f"\nKey: '{key}'")
                print(f"  Type: {type(value)}")
                
                if torch.is_tensor(value):
                    print(f"  Shape: {value.shape}")
                    print(f"  Dtype: {value.dtype}")
                    print(f"  Device: {value.device}")
                    print(f"  Requires grad: {value.requires_grad}")
                    print(f"  Memory usage: {value.numel() * value.element_size() / (1024*1024):.2f} MB")
                    
                    # Print some basic statistics for numeric tensors
                    if value.dtype in [torch.float32, torch.float64, torch.float16]:
                        print(f"  Min: {value.min().item():.6f}")
                        print(f"  Max: {value.max().item():.6f}")
                        print(f"  Mean: {value.mean().item():.6f}")
                        print(f"  Std: {value.std().item():.6f}")
                elif isinstance(value, (list, tuple)):
                    print(f"  Length: {len(value)}")
                    if len(value) > 0:
                        print(f"  First element type: {type(value[0])}")
                else:
                    print(f"  Value: {value}")
        
        elif torch.is_tensor(data):
            print("\n--- Single Tensor ---")
            print(f"Shape: {data.shape}")
            print(f"Dtype: {data.dtype}")
            print(f"Device: {data.device}")
            print(f"Requires grad: {data.requires_grad}")
            print(f"Memory usage: {data.numel() * data.element_size() / (1024*1024):.2f} MB")
            
            if data.dtype in [torch.float32, torch.float64, torch.float16]:
                print(f"Min: {data.min().item():.6f}")
                print(f"Max: {data.max().item():.6f}")
                print(f"Mean: {data.mean().item():.6f}")
                print(f"Std: {data.std().item():.6f}")
        
        else:
            print(f"\n--- Other Data Type ---")
            print(f"Content: {data}")
            
    except Exception as e:
        print(f"Error loading file: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python pt_probe.py <path_to_pt_file>")
        sys.exit(1)
    
    file_path = sys.argv[1]
    probe_pt_file(file_path)