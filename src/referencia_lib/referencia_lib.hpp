//
// Created by Edumarek on 06/02/2023.
//

#ifndef REFERENCIA_LIB_HPP
#define REFERENCIA_LIB_HPP

#include "../matriz_lib/matriz_lib.hpp"

#define PpIi 3.14159

typedef _ponto (*refType)(_ponto, _ponto, _ponto);

struct Referencia{
    _ponto modulo = 0;
    _ponto angulo = 0;
    _ponto velocidade = 0.1;
    refType entrada;

    _ponto moduloMax=1;
    _ponto moduloMin=0;
    
    Referencia(_ponto _moduloMin, _ponto _moduloMax, refType _entrada);
    void atualiza_referencia();

};

_ponto degrau_unitario(_ponto m, _ponto a, _ponto v);
_ponto rampa(_ponto m, _ponto a, _ponto v);
_ponto onda_quadrada(_ponto m, _ponto a, _ponto v);
_ponto onda_triangular(_ponto m, _ponto a, _ponto v);
_ponto onda_senoidal(_ponto m, _ponto a, _ponto v);



#endif //REFERENCIA_LIB_HPP
