# Guia de Configuração e Uso: Projeto ROS 2 no Windows com WSL2

Este guia detalha todos os passos para configurar um ambiente de desenvolvimento ROS 2 completo no Windows, utilizando WSL2, Docker e VS Code. Ao final, você terá um contêiner com ROS 2 Humble, Gazebo e RViz funcionando com aceleração gráfica.

-----

## 1\. Instalação de Pré-requisitos

### 1.1 Habilitar o WSL2

Execute os seguintes comandos no **PowerShell (como Administrador)** para instalar e configurar o WSL2.

```powershell
wsl --install
wsl --set-default-version 2
```

### 1.2 Instalar o Docker Desktop

Baixe e instale o **[Docker Desktop para Windows](https://www.docker.com/products/docker-desktop/)**.

Após a instalação, configure o Docker para usar o backend do WSL2:

1.  Vá em **Settings \> General**.
2.  Marque a opção **Use the WSL 2 based engine**.
3.  Vá em **Settings \> Resources \> WSL Integration**.
4.  Habilite a integração para a sua distribuição Linux (ex: `Ubuntu-22.04`).

### 1.3 Instalar uma Distribuição Linux (Ubuntu 22.04)

1.  Abra a **Microsoft Store**.
2.  Procure e instale o **Ubuntu 22.04 LTS**.
3.  Inicie o Ubuntu pelo Menu Iniciar e configure seu nome de usuário e senha.

### 1.4 Configurar o Servidor X (Para Apps com GUI)

Para que aplicativos gráficos como Gazebo e RViz funcionem, precisamos de um X Server no Windows.

1.  Baixe e instale o **[VcXsrv](https://sourceforge.net/projects/vcxsrv/)**.
2.  Após instalar, procure por **XLaunch** no Menu Iniciar e execute-o com as seguintes configurações:
      * Selecione **"Multiple windows"**.
      * Display number: **-1**.
      * Marque a opção **"Disable access control"** (Muito importante\!).
      * Finalize e salve a configuração para uso futuro.

-----

## 2\. Preparação do Projeto no WSL2

### 2.1 Clonar o Repositório

Abra o terminal do seu **Ubuntu (WSL2)** e execute:

```bash
git clone [https://github.com/felipiadenildo/SSC0712-Trabalho-PRM.git](https://github.com/felipiadenildo/SSC0712-Trabalho-PRM.git)
cd SSC0712-Trabalho-PRM
```

### 2.2 Configurar o Encaminhamento X11

Para que as janelas do contêiner apareçam no Windows, configure a variável de ambiente `DISPLAY`. Adicione as seguintes linhas ao final do seu arquivo `~/.bashrc` dentro do WSL2:

```bash
export DISPLAY=$(awk '/nameserver / {print $2}' /etc/resolv.conf):0
export LIBGL_ALWAYS_INDIRECT=1
```

Depois, recarregue o terminal ou execute:

```bash
source ~/.bashrc
```

### 2.3 Conceder Permissões de Execução

Ainda no terminal do WSL2, conceda permissão de execução aos scripts do projeto:

```bash
chmod +x .devcontainer/*.sh
```

-----

## 3\. Construir e Executar o Contêiner

### 3.1 Abrir o Projeto no VS Code

1.  Abra o **VS Code**.
2.  Instale a extensão **Remote - WSL**.
3.  Pressione `Ctrl+Shift+P` e procure por **"Remote-WSL: Reopen Folder in WSL"**.
4.  Navegue e abra a pasta do seu projeto `SSC0712-Trabalho-PRM`.

### 3.2 Reconstruir o Contêiner

Com o projeto aberto no ambiente WSL, o VS Code deve sugerir reabrir o projeto em um Dev Container. Se não o fizer:

1.  Pressione `Ctrl+Shift+P`.
2.  Execute o comando **"Dev Containers: Rebuild Container"**.
3.  Aguarde a construção da imagem Docker (pode levar de 5 a 10 minutos na primeira vez).

### 3.3 Verificar o Encaminhamento X11

Com o contêiner em execução, abra um terminal do VS Code (que agora está *dentro* do contêiner) e execute:

```bash
xeyes
```

Se uma janela com dois olhos que seguem o cursor do mouse aparecer, o encaminhamento gráfico está funcionando\! 🎉

-----

## 4\. Executando as Aplicações ROS 2

Use o script `run-app.sh` para controlar a simulação.

### 4.1 Iniciar a Simulação

```bash
./run-app.sh sim
```

### 4.2 Iniciar o Controle do Robô

```bash
./run-app.sh ctrl
```

### 4.3 Parar Tudo

```bash
./run-app.sh kill
```

-----

## 5\. Solução de Problemas Comuns

### 5.1 Apps com GUI não aparecem

  * Garanta que o **VcXsrv** esteja rodando no Windows (verifique o ícone na bandeja do sistema).
  * Verifique a variável `DISPLAY` dentro do WSL2 com `echo $DISPLAY`. Ela deve mostrar um IP, como `172.28.80.1:0`.
  * Tente desabilitar temporariamente o Firewall do Windows.

### 5.2 Aceleração Gráfica não funciona

  * Garanta que você tenha o **driver da NVIDIA para WSL2** instalado no Windows.
  * Verifique se o driver está acessível com o comando `nvidia-smi` no terminal do WSL2.

### 5.3 Problemas de Permissão do Docker

Se receber erros de "Permission denied" ao tentar construir o contêiner:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

-----

## 6\. Notas Finais

✅ **Funciona:**

  * ROS 2 Humble
  * Gazebo (com aceleração de GPU se os drivers NVIDIA estiverem instalados)
  * RViz2
  * Aplicativos com GUI via X11 (VcXsrv)

⚠️ **Limitações:**

  * O desempenho pode ser ligeiramente inferior ao de um sistema Linux nativo.
  * O uso de dispositivos USB (como joysticks) requer configuração extra de encaminhamento.

### Resumo dos Arquivos Principais e Suas Funções

| Arquivo             | Propósito                                                | Localização       |
| ------------------- | -------------------------------------------------------- | ----------------- |
| `Dockerfile`        | Define o ambiente do contêiner (pacotes, etc.)           | `.devcontainer/`  |
| `devcontainer.json` | Configura o Dev Container do VS Code (extensões, etc.)   | `.devcontainer/`  |
| `setup-env.sh`      | Configura o ambiente ROS e o workspace dentro do contêiner | `.devcontainer/`  |
| `run-app.sh`        | Controla a execução das aplicações ROS 2 (sim, ctrl, etc.) | `.devcontainer/`  |

Este setup garante uma experiência de desenvolvimento ROS 2 fluida no Windows com suporte a GUI. 🚀

```
</immersive>
```