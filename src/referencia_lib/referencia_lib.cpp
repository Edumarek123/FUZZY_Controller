//
// Created by Edumarek on 06/02/2023.
//

#include "referencia_lib.hpp"

Referencia::Referencia(_ponto _moduloMin, _ponto _moduloMax, refType _entrada)
{
    moduloMin = _moduloMin;
    moduloMax = _moduloMax;

    entrada = _entrada;
}

void Referencia::atualiza_referencia()
{
    if (entrada == degrau_unitario || entrada == onda_quadrada || entrada == onda_senoidal || entrada == onda_triangular)
        modulo = entrada(moduloMax, angulo, velocidade);
    else
        modulo = entrada(modulo, angulo, velocidade);

    angulo += velocidade;

    if (modulo > moduloMax)
        modulo = moduloMax;

    if (modulo < moduloMin)
        modulo = moduloMin;

    if (angulo > 2 * PpIi)
        angulo = 0;
    if (angulo < 0)
        angulo = 2 * PpIi;
}

_ponto degrau_unitario(_ponto m, _ponto a, _ponto v)
{
    return m * v;
}

_ponto rampa(_ponto m, _ponto a, _ponto v)
{
    return m + v;
}

_ponto onda_quadrada(_ponto m, _ponto a, _ponto v)
{
    if (a < PpIi)
        return m;
    return 0;
}

_ponto onda_triangular(_ponto m, _ponto a, _ponto v)
{
    a = (a / (2 * PpIi));

    if (a < 0.5)
        return 2 * a * m;

    return m - (2 * (a - 0.5) * m);
}

_ponto onda_senoidal(_ponto m, _ponto a, _ponto v)
{
    return (m / 2) + ((m / 2) * sin(a));
}
