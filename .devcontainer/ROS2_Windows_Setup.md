```markdown
# Guia Completo de Configuração ROS 2 no Windows (WSL2 + Docker)

## 🔧 1. Pré-requisitos Essenciais

### 1.1 Ativação do WSL2 (PowerShell Admin)
```powershell
# Executar como Administrador
wsl --install
wsl --set-default-version 2
wsl --shutdown
```

### 1.2 Instalação do Docker Desktop
1. Baixe o [Docker Desktop for Windows](https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe)
2. Após instalação:
   - Ative **WSL2 backend**:  
     `Settings > General > Use WSL2 based engine`
   - Configure integração:  
     `Settings > Resources > WSL Integration > Ativar sua distro (ex: Ubuntu-22.04)`

### 1.3 Instalação do Ubuntu 22.04
1. Abra a Microsoft Store e pesquise "Ubuntu 22.04 LTS"
2. Clique em "Instalar"
3. Ao concluir, inicie pelo Menu Iniciar e:
   ```bash
   # Na primeira execução:
   Digite novo nome de usuário UNIX: [seu_user]
   New password: [sua_senha]
   Confirm password: [sua_senha]
   ```

### 1.4 Configuração do X Server (VcXsrv)
1. Baixe e instale o [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Inicie o **XLaunch** (via Menu Iniciar) e configure:
   - Display settings: "Multiple windows" (Display number: `-1`)
   - Client startup: Marque "Disable access control"
   - Extra settings: Deixe os padrões
3. Salve a configuração (opcional)

## 🛠️ 2. Configuração do Ambiente WSL2

### 2.1 Configuração do `.bashrc`
```bash
# Adicione ao final do arquivo ~/.bashrc:
echo -e "\n# Configurações ROS\n" >> ~/.bashrc
echo "export DISPLAY=\$(awk '/nameserver / {print \$2}' /etc/resolv.conf):0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc
echo "export GTK_IM_MODULE=ibus" >> ~/.bashrc  # Para input methods
source ~/.bashrc
```

## 🐋 3. Configuração do Projeto

### 3.1 Clone do Repositório
```bash
git clone https://github.com/felipiadenildo/SSC0712-Trabalho-PRM.git
cd SSC0712-Trabalho-PRM
```

### 3.2 Permissões de Execução
```bash
chmod +x .devcontainer/*.sh
```

## 💻 4. Configuração do VS Code

1. Instale as extensões essenciais:
   - Remote - WSL
   - ROS (by Microsoft)
   - Python (by Microsoft)
2. Abra o projeto no WSL:
   - `Ctrl+Shift+P` > "Remote-WSL: Reopen Folder in WSL"
3. Rebuild do container:
   - `Ctrl+Shift+P` > "Dev Containers: Rebuild Container"

## 🚀 5. Execução do Projeto

### 5.1 Iniciar Simulação
```bash
./run-app.sh sim
# Verifique no VcXsrv a janela do Gazebo
```

### 5.2 Iniciar Controle
```bash
./run-app.sh ctrl
```

### 5.3 Parar Todos os Processos
```bash
./run-app.sh kill
```

## 🔍 6. Troubleshooting Detalhado

### 6.1 Problemas Comuns
| Sintoma | Solução |
|---------|---------|
| Gazebo não abre | Verifique se o VcXsrv está rodando e firewall desativado |
| Erros de GPU | Execute `nvidia-smi` no WSL para verificar drivers |
| DBUS errors | Reinicie o container com `docker-compose down && docker-compose up` |

### 6.2 Verificação de Ambiente
```bash
# Teste X11 forwarding:
xeyes

# Verifique GPU:
glxinfo | grep "OpenGL renderer"

# Verifique variáveis críticas:
printenv | grep -E 'DISPLAY|LIBGL|DBUS'
```

## 📌 7. Notas Finais

- **Performance**: Para melhor desempenho com GPU NVIDIA:
  ```bash
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  ```

- **Atualização**: Sempre execute no WSL:
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```

- **Problemas persistentes?** Recrie todo o ambiente:
  ```powershell
  wsl --unregister Ubuntu-22.04
  wsl --install
  ```

Este guia cobre todos os passos desde a instalação até a execução estável no Windows, incluindo soluções para os problemas mais comuns. Todos os comandos foram testados em ambiente Windows 10/11 com WSL2.