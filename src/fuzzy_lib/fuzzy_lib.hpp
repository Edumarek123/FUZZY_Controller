//
// Created by Edumarek on 30/03/2023.
//

#ifndef FUZZY_LIB_HPP
#define FUZZY_LIB_HPP

#include <Arduino.h>

#ifndef _ponto
#define _ponto double
#endif

#define N_Regras 5

struct ControladorFuzzy
{
    // parametros controlador
    _ponto *centrosEk;
    _ponto *centrosDEk;
    _ponto *centrosDU;
    _ponto *baseEk;
    _ponto *baseDEk;
    _ponto *miE;
    _ponto *miDE;
    _ponto *miDU;
    _ponto alturaEk = 1;
    _ponto alturaDEk = 1;
    // limites
    _ponto min_Uk = 0;
    _ponto max_Uk = 1;
    // variaveis controlador
    _ponto Ek[2] = {0, 0};     // erro
    _ponto DEk = 0;            // erro
    _ponto Uk[2] = {0.1, 0.1}; // acao de controle
    _ponto DUk = 0;            // acao de controle
    // variaveis auxiliares
    bool calcularTempoExecucao = false;
    unsigned long tExecucao = 0; // ms
    bool imprimirParametros = false;

    ControladorFuzzy() = delete;
    ControladorFuzzy(_ponto min_uK, _ponto max_uK);
    ControladorFuzzy(const ControladorFuzzy &c);
    ~ControladorFuzzy();

    _ponto Calcula_Acao_Controle(_ponto referencia, _ponto saida);
};

_ponto max_value(_ponto A, _ponto B);

#endif // FUZZY_PID_LIB_HPP
