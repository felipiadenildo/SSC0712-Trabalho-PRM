# Guia de Configuração e Execução: ROS 2 com Docker e WSL2

Este guia detalha o processo completo para configurar e executar este projeto ROS 2 em um ambiente Windows, utilizando o WSL2 (Subsistema do Windows para Linux) e Docker.

---

## 🏁 Parte 1: Configuração do Ambiente Windows (Apenas uma vez)

Estes passos preparam seu computador para rodar o projeto. Você só precisa fazê-los uma única vez.

### 1.1. Instalar e Ativar o WSL2

O WSL2 permite executar um ambiente Linux diretamente no Windows.

1.  Abra o **PowerShell como Administrador**.
2.  Execute o seguinte comando para instalar o WSL2 e a distribuição Ubuntu 22.04:
    ```powershell
    wsl --install -d Ubuntu-22.04
    ```
3.  Reinicie o seu computador quando solicitado.
4.  Após reiniciar, o terminal do Ubuntu será aberto. Crie um nome de usuário e uma senha para o seu ambiente Linux.

### 1.2. Instalar o Docker Desktop

O Docker irá gerenciar nosso ambiente de desenvolvimento isolado (contêiner).

1.  Baixe e instale o [Docker Desktop for Windows](https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe).
2.  Durante a instalação, certifique-se de que a opção **"Use WSL 2 based engine"** esteja marcada.
3.  Após a instalação, abra o Docker Desktop, vá para **Settings > Resources > WSL Integration** e ative a integração para a sua distribuição `Ubuntu-22.04`.

### 1.3. Instalar um X Server (VcXsrv)

Aplicações com interface gráfica (GUI) como o Gazebo e o RViz, que rodam no Linux (dentro do Docker), precisam de um "servidor de janelas" no Windows para serem exibidas.

1.  Baixe e instale o [VcXsrv](https://sourceforge.net/projects/vcxsrv/).
2.  Inicie o **XLaunch** pelo Menu Iniciar e configure-o da seguinte forma:
    * **Display settings**: Selecione "Multiple windows" e deixe o "Display number" como `-1`.
    * **Client startup**: Selecione "Start no client".
    * **Extra settings**: Marque as opções **"Clipboard"**, **"Primary Selection"** e, o mais importante, **"Disable access control"**.
3.  Conclua a configuração. Um ícone do VcXsrv aparecerá na sua bandeja do sistema, indicando que ele está em execução.

### 1.4. Instalar e Configurar o VS Code

O VS Code é o editor que usaremos para desenvolver e interagir com o contêiner.

1.  Instale o [Visual Studio Code](https://code.visualstudio.com/).
2.  Dentro do VS Code, vá para a aba de Extensões (`Ctrl+Shift+X`) e instale a extensão **"WSL"** da Microsoft.

---

## 🚀 Parte 2: Executando o Projeto

Com o ambiente configurado, siga estes passos para executar o projeto.

### 2.1. Abrir o Projeto no WSL

1.  Abra seu terminal **Ubuntu** (pelo Menu Iniciar).
2.  Clone o repositório para dentro do seu ambiente WSL:
    ```bash
    git clone [https://github.com/felipiadenildo/SSC0712-Trabalho-PRM.git](https://github.com/felipiadenildo/SSC0712-Trabalho-PRM.git)
    cd SSC0712-Trabalho-PRM
    ```
3.  Abra o projeto no VS Code:
    ```bash
    code .
    ```
    O VS Code será iniciado no Windows, mas conectado ao seu sistema de arquivos Linux.

### 2.2. Iniciar o Ambiente de Desenvolvimento (Dev Container)

O VS Code detectará a pasta `.devcontainer` e sugerirá reabrir o projeto em um contêiner.

1.  Um pop-up aparecerá no canto inferior direito. Clique em **"Reopen in Container"**.
2.  Aguarde enquanto o Docker constrói a imagem e inicia o contêiner. Este processo pode demorar alguns minutos na primeira vez.
3.  Quando terminar, o VS Code estará conectado diretamente ao seu ambiente de desenvolvimento ROS 2, pronto para uso.

### 2.3. Compilar e Executar a Aplicação

Agora, com tudo pronto, vamos compilar e rodar a simulação.

1.  Abra um novo terminal dentro do VS Code (`Ctrl+'` ou `Terminal > New Terminal`). Você já estará dentro do contêiner.
2.  **Execute o script de configuração e compilação.** Este script instala as dependências do ROS e compila seu código.
    ```bash
    # Dentro do terminal do VS Code (no contêiner)
    ./.devcontainer/setup-env.sh
    ```
3.  **Ative o ambiente do seu workspace.** Após a compilação, você precisa "ativar" os pacotes que foram criados.
    ```bash
    source install/setup.bash
    ```
    > **Dica:** O alias `s_ws` foi criado para fazer isso mais rápido. Você pode digitar apenas `s_ws`.

4.  **Execute a aplicação usando o script de controle:**
    * Para iniciar a simulação (Gazebo):
        ```bash
        run-app.sh sim
        ```
    * Para iniciar os nós de controle (RViz, etc.):
        ```bash
        run-app.sh ctrl
        ```
    * Para parar todos os processos:
        ```bash
        run-app.sh kill
        ```

---

## 🔍 Solução de Problemas (Troubleshooting)

* **Erro `command not found: run-app.sh`:** Você esqueceu de rodar `source install/setup.bash` (ou o alias `s_ws`) no seu terminal.
* **Janelas do Gazebo/RViz não abrem:**
    1.  Verifique se o **VcXsrv** está em execução no Windows.
    2.  Confirme se a opção **"Disable access control"** foi marcada durante a configuração do VcXsrv.
    3.  Verifique se o seu Firewall do Windows não está bloqueando a conexão.
* **Problemas persistentes e inexplicáveis:** O ambiente Docker pode ter se corrompido. A solução mais rápida é reconstruí-lo:
    1.  No VS Code, pressione `Ctrl+Shift+P`.
    2.  Digite e selecione **"Dev Containers: Rebuild Container"**.
