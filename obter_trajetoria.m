function [x,y,t] = obter_trajetoria(velocidade,a,taxa_amostragem)

% Cálculo do tempo total de percurso
tempo_total = (6.09722*a) / velocidade;

% Cálculo do número de pontos de amostragem
numero_pontos = round(tempo_total * taxa_amostragem);

% Vetores para armazenar os pontos da trajetória e o tempo

% Cálculo dos pontos da trajetória
t = linspace(0, tempo_total, numero_pontos);

% Parâmetro do movimento no formato de oito
parametro = (2 * pi * t) / (tempo_total / 2);

% Cálculo das coordenadas x e y
x = a .* sin(parametro);
y = a .* sin(parametro) .* cos(parametro);

    
end

