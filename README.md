﻿# Monophonic music player

Este é o repositório de um sistema embarcado que reproduz duas músicas monofonias que foi realizado em parceria com os professores Rafael Corsi e Eduardo Marossi para a matéria Computação Embarcada no primeiro semestre de 2020 (2020.1).


# Passos para executar o projeto.

1. É necessário a instalação do software Atmel Studio, que só pode ser executado em WindowsOS.
2. Clone o repositório monophonic-music com o código abaixo:
`git clone https://github.com/aronfelipe/monophonic-music`
3. Abra o projeto no Atmel Studio (entre no Atmel Studio clique na aba "File" em seguinda siga "Open" -> "Project/Solution..." depois entre no repositório pelo explorer clique em "SAME70-OLED-BUZZER-MUSICAL" e escolha o arquivo "SAME70-OLED-BUZZER-MUSICAL.atsln", ícone vermelho).
4. Agora que você ja tem o código precisa conectar o Buzzer (o menos em qualquer GROUND do SAME70) ao microcontrolador e o OLED no EXT1 conforme os detalhes técnicos mais a frente (você pode checar a foto se ficar com dúvida).
5. Agora é necessário fazer o build do projeto clique na aba "Build" e depois na opção "Rebuild Solution".
6. Depois clique na setinha com a legenda "Start without Debugging" que é a seta verde mais a direita perto de um selecionador "Debug".
7. Desfrute de música monofonica.

## Detalhes técnicos

```mermaid
graph LR
A[SAME70] --> B((BUZZER))
A --> C(OLED)
```

A conexão do buzzer foi feita de acordo com outros tutoriais mas segue o padrão, o menos vai no pino GND 2 do lado esquerdo do SAME70 e o positivo vai no pino PC19. Já o OLED é conectado diretamente no EXT1 "consumindo" todos os pinos do EXT1

## Observações

Para começar a tocar músicas é necessário clicar no botão 2 do OLED identificado como "BUTTON 2". Para pausar é necessário clicar no botão "BUTTON 1". Para retomar a música onde estava ou seja o "resume" da música é só clicar novamente no botão "BUTTON 1". Para mudar de música é necessário pausar a música primeiro com o "BUTTON 1" e depois clicar no botão "BUTTON 3" do OLED e a próxima música começa a tocar instantaneamente.

## Vídeo do protótipo funcionando

https://youtu.be/l1w_eHXLvp8
