//
// Created by Edumarek on 30/03/2023.
//

#include "fuzzy_lib.hpp"

//------------------CONSTRUTORES&DESTRUTORES-------------------//
ControladorFuzzy::ControladorFuzzy(_ponto min_uK, _ponto max_uK)
{
    min_Uk = min_uK;
    max_Uk = max_uK;

    centrosEk = (_ponto *)malloc(N_Regras * sizeof(_ponto));
    centrosDEk = (_ponto *)malloc(N_Regras * sizeof(_ponto));
    centrosDU = (_ponto *)malloc(N_Regras * sizeof(_ponto));
    baseEk = (_ponto *)malloc(N_Regras * sizeof(_ponto));
    baseDEk = (_ponto *)malloc(N_Regras * sizeof(_ponto));
    miE = (_ponto *)malloc(N_Regras * sizeof(_ponto));
    miDE = (_ponto *)malloc(N_Regras * sizeof(_ponto));
    miDU = (_ponto *)malloc(N_Regras * sizeof(_ponto));
}

ControladorFuzzy::ControladorFuzzy(const ControladorFuzzy &c)
{
}

ControladorFuzzy::~ControladorFuzzy()
{
    free(centrosEk);
    free(centrosDEk);
    free(centrosDU);
    free(baseEk);
    free(baseDEk);
    free(miE);
    free(miDE);
    free(miDU);
}

//---------------------------METODOS---------------------------//
_ponto ControladorFuzzy::Calcula_Acao_Controle(_ponto referencia, _ponto saida)
{
    unsigned long tInicio = micros();
    // Calcula erro
    Ek[0] = referencia - saida;
    DEk = Ek[0] - Ek[1];

    // Fuzzyficacao
    for (int i = 0; i < N_Regras; i++)
    {
        miE[i] = 0;
        miDE[i] = 0;
        miDU[i] = 0;

        _ponto modulo1 = abs(Ek[0] - centrosEk[i]);
        if (modulo1 <= baseEk[i])
        {
            miE[i] = alturaEk - (modulo1 / baseEk[i]);
        }

        _ponto modulo2 = abs(DEk - centrosDEk[i]);
        if (modulo2 <= baseDEk[i])
        {
            miDE[i] = alturaDEk - (modulo2 / baseDEk[i]);
        }
    }

    // Regras de inferencia
    for (int i = 0; i < N_Regras; i++)
    {
        if (miE[i] != 0 && miDE[2] != 0)
            miDU[i] = max_value(miE[i], miDE[2]);
    }

    for (int i = 0; i < N_Regras; i++)
    {
        if (miE[2] != 0 && miDE[i] != 0)
            miDU[i] = max_value(miE[2], miDE[i]);
    }
    // Defuzzyficacao por centroides
    DUk = 0;
    _ponto SmiDU = 0;
    for (int i = 0; i < N_Regras; i++)
    {
        SmiDU += miDU[i];
        DUk += miDU[i] * centrosDU[i];
    }
    DUk /= SmiDU;

    if (DUk > centrosDU[N_Regras - 1])
        DUk = centrosDU[N_Regras - 1];
    if (DUk <= centrosDU[0])
        DUk = centrosDU[0];

    // Calculo acao de controle
    if (DUk == DUk)
        Uk[0] = Uk[1] + DUk;

    // Limitador acao controle
    if (Uk[0] > max_Uk)
        Uk[0] = max_Uk;
    if (Uk[0] <= min_Uk)
        Uk[0] = min_Uk;

    // Atualiza erros
    Ek[1] = Ek[0];

    // Atualiza acoes de controle
    Uk[1] = Uk[0];

    if (calcularTempoExecucao)
        tExecucao = micros() - tInicio;

    return Uk[0];
}
//---------------------------FUNCOES-AUXILIARES---------------------------//

_ponto max_value(_ponto A, _ponto B)
{
    if (A > B)
        return A;
    else
        return B;
}