import json
from pathlib import Path

import joblib
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from torch.utils.data import DataLoader, TensorDataset
from tqdm import tqdm


class ReSkinMLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.layer1 = nn.Linear(15, 200)
        self.relu1 = nn.ReLU()
        self.layer2 = nn.Linear(200, 200)
        self.layer3 = nn.Linear(200, 40)
        self.layer4 = nn.Linear(40, 200)
        self.relu4 = nn.ReLU()
        self.layer5 = nn.Linear(200, 200)
        self.relu5 = nn.ReLU()
        self.output_layer = nn.Linear(200, 3)

    def forward(self, x):
        x = self.relu1(self.layer1(x))
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.relu4(self.layer4(x))
        x = self.relu5(self.layer5(x))
        x = self.output_layer(x)
        return x


def load_all_data(data_dir: Path):
    json_paths = sorted(data_dir.rglob("*.json"))

    X_list = []
    y_list = []

    for file_path in json_paths:
        with file_path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        for sample in data:
            tactile_flat = [item for row in sample["tactile_data"] for item in row]
            ft = sample["ft_data"]
            X_list.append(tactile_flat)
            y_list.append([ft[0], ft[1], ft[2]])

    if not X_list:
        return np.empty((0, 15), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

    return np.array(X_list, dtype=np.float32), np.array(y_list, dtype=np.float32)


def compute_metrics(y_true: np.ndarray, y_pred: np.ndarray):
    out = {}

    for i, axis_name in enumerate(["fx", "fy", "fz"]):
        err = y_pred[:, i] - y_true[:, i]
        out[axis_name] = {
            "mae": float(np.mean(np.abs(err))),
            "rmse": float(np.sqrt(np.mean(err ** 2))),
        }

    vec_err = np.linalg.norm(y_pred - y_true, axis=1)
    out["vector"] = {
        "mae": float(np.mean(vec_err)),
        "rmse": float(np.sqrt(np.mean(vec_err ** 2))),
    }

    return out


def compute_accuracy(y_true: np.ndarray, y_pred: np.ndarray, tol_n: float):
    out = {}

    for i, axis_name in enumerate(["fx", "fy", "fz"]):
        abs_err = np.abs(y_pred[:, i] - y_true[:, i])
        out[axis_name] = float(np.mean(abs_err <= tol_n))

    out["all_axes"] = float(np.mean(np.abs(y_pred - y_true) <= tol_n))
    out["sample_all3"] = float(np.mean(np.all(np.abs(y_pred - y_true) <= tol_n, axis=1)))

    return out


def main():
    current_dir = Path(__file__).parent

    data_dir = current_dir / "data" 

    model_dir = current_dir / "model" / "3d"
    cv_plot_dir = model_dir / "loss_plots"
    model_dir.mkdir(parents=True, exist_ok=True)
    cv_plot_dir.mkdir(parents=True, exist_ok=True)

    if not data_dir.exists():
        raise FileNotFoundError(f"Data folder not found: {data_dir}")

    print("Loading all data from dataset folder...")
    X, y = load_all_data(data_dir=data_dir)

    print(f"Total samples loaded: {len(X)}")
    print(f"  source folder:  {data_dir}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Training on: {device}")

    np.random.seed(42)
    torch.manual_seed(42)

    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    batch_size = 32
    num_epochs = 100
    learning_rate = 1e-3
    acc_tolerance_n = 0.5

    fold_results = []

    print("\nStarting 5-Fold Cross-Validation...")
    print("=" * 80)

    for fold_idx, (train_idx, val_idx) in enumerate(kf.split(X)):
        print(f"\nFold {fold_idx + 1}/5")
        print("-" * 80)

        X_train, X_val = X[train_idx], X[val_idx]
        y_train, y_val = y[train_idx], y[val_idx]

        scaler_X = StandardScaler()
        X_train_scaled = scaler_X.fit_transform(X_train)
        X_val_scaled = scaler_X.transform(X_val)

        X_train_tensor = torch.tensor(X_train_scaled, dtype=torch.float32)
        y_train_tensor = torch.tensor(y_train, dtype=torch.float32)
        X_val_tensor = torch.tensor(X_val_scaled, dtype=torch.float32)
        y_val_tensor = torch.tensor(y_val, dtype=torch.float32)

        train_dataset = TensorDataset(X_train_tensor, y_train_tensor)
        val_dataset = TensorDataset(X_val_tensor, y_val_tensor)

        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

        model = ReSkinMLP().to(device)
        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)

        train_losses = []
        val_losses = []

        for epoch in tqdm(range(num_epochs), desc=f"Fold {fold_idx + 1} epochs", leave=False):
            model.train()
            train_loss = 0.0

            for batch_X, batch_y in train_loader:
                batch_X = batch_X.to(device)
                batch_y = batch_y.to(device)

                pred = model(batch_X)
                loss = criterion(pred, batch_y)

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                train_loss += loss.item() * batch_X.size(0)

            train_loss /= len(train_loader.dataset)

            model.eval()
            val_loss = 0.0
            with torch.no_grad():
                for batch_X, batch_y in val_loader:
                    batch_X = batch_X.to(device)
                    batch_y = batch_y.to(device)

                    pred = model(batch_X)
                    loss = criterion(pred, batch_y)
                    val_loss += loss.item() * batch_X.size(0)

            val_loss /= len(val_loader.dataset)

            train_losses.append(train_loss)
            val_losses.append(val_loss)

            if (epoch + 1) % 10 == 0:
                print(
                    f"  Epoch [{epoch + 1}/{num_epochs}] "
                    f"Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f}"
                )

        curve_path = cv_plot_dir / f"fold_{fold_idx + 1}_curve.png"
        plt.figure(figsize=(8, 5))
        plt.plot(range(1, num_epochs + 1), train_losses, label="Train Loss", linewidth=2)
        plt.plot(range(1, num_epochs + 1), val_losses, label="Validation Loss", linewidth=2)
        plt.xlabel("Epoch")
        plt.ylabel("MSE Loss")
        plt.title(f"Fold {fold_idx + 1} Training vs Validation Loss")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(curve_path, dpi=150)
        plt.close()

        model.eval()
        with torch.no_grad():
            val_pred = model(X_val_tensor.to(device)).cpu().numpy()

        metrics = compute_metrics(y_true=y_val, y_pred=val_pred)
        acc = compute_accuracy(y_true=y_val, y_pred=val_pred, tol_n=acc_tolerance_n)
        fold_results.append({"metrics": metrics, "accuracy": acc})

        print("Validation metrics:")
        print(f"  Fx  MAE={metrics['fx']['mae']:.6f}, RMSE={metrics['fx']['rmse']:.6f}")
        print(f"  Fy  MAE={metrics['fy']['mae']:.6f}, RMSE={metrics['fy']['rmse']:.6f}")
        print(f"  Fz  MAE={metrics['fz']['mae']:.6f}, RMSE={metrics['fz']['rmse']:.6f}")
        print(f"  |F| MAE={metrics['vector']['mae']:.6f}, RMSE={metrics['vector']['rmse']:.6f}")
        print(f"Validation accuracy @ |error| <= {acc_tolerance_n:.2f}N:")
        print(f"  Fx ACC={acc['fx']:.4f}")
        print(f"  Fy ACC={acc['fy']:.4f}")
        print(f"  Fz ACC={acc['fz']:.4f}")
        print(f"  All-axis element ACC={acc['all_axes']:.4f}")
        print(f"  Sample all-3 ACC={acc['sample_all3']:.4f}")

    print("\n" + "=" * 80)
    print("Cross-validation summary (mean ± std):")

    for key in ["fx", "fy", "fz", "vector"]:
        maes = [r["metrics"][key]["mae"] for r in fold_results]
        rmses = [r["metrics"][key]["rmse"] for r in fold_results]
        print(
            f"  {key.upper():<8} "
            f"MAE={np.mean(maes):.6f} ± {np.std(maes):.6f}, "
            f"RMSE={np.mean(rmses):.6f} ± {np.std(rmses):.6f}"
        )

    print(f"\nCross-validation accuracy summary @ |error| <= {acc_tolerance_n:.2f}N:")
    for key in ["fx", "fy", "fz", "all_axes", "sample_all3"]:
        vals = [r["accuracy"][key] for r in fold_results]
        print(f"  {key.upper():<14} ACC={np.mean(vals):.4f} ± {np.std(vals):.4f}")

    print("\n" + "=" * 80)
    print("Final training on full dataset...")

    final_scaler = StandardScaler()
    X_full_scaled = final_scaler.fit_transform(X)

    X_full_tensor = torch.tensor(X_full_scaled, dtype=torch.float32)
    y_full_tensor = torch.tensor(y, dtype=torch.float32)

    full_dataset = TensorDataset(X_full_tensor, y_full_tensor)
    full_loader = DataLoader(full_dataset, batch_size=batch_size, shuffle=True)

    final_model = ReSkinMLP().to(device)
    final_criterion = nn.MSELoss()
    final_optimizer = optim.Adam(final_model.parameters(), lr=learning_rate)

    for epoch in tqdm(range(num_epochs), desc="Final training", leave=False):
        final_model.train()
        epoch_loss = 0.0

        for batch_X, batch_y in full_loader:
            batch_X = batch_X.to(device)
            batch_y = batch_y.to(device)

            pred = final_model(batch_X)
            loss = final_criterion(pred, batch_y)

            final_optimizer.zero_grad()
            loss.backward()
            final_optimizer.step()

            epoch_loss += loss.item() * batch_X.size(0)

        epoch_loss /= len(full_loader.dataset)

    print(f"Final training loss: {epoch_loss:.6f}")

    final_model_path = model_dir / "model.pth"
    final_scaler_path = model_dir / "scaler.pkl"

    torch.save(final_model.state_dict(), final_model_path)
    joblib.dump(final_scaler, final_scaler_path)

    print(f"Saved model:  {final_model_path}")
    print(f"Saved scaler: {final_scaler_path}")


if __name__ == "__main__":
    main()
