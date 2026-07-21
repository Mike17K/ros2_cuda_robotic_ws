# 1. Προσθήκη του αποθετηρίου της NVIDIA
curl -fsSL https://github.io | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://github.io | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Ενημέρωση λίστας πακέτων και εγκατάσταση
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit isaac-ros-cli

# 3. Ρύθμιση του Docker να χρησιμοποιεί το NVIDIA runtime
sudo nvidia-container-toolkit remote-configure --runtime=docker

# 4. Επανεκκίνηση της υπηρεσίας Docker για να εφαρμοστούν οι αλλαγές
sudo systemctl restart docker