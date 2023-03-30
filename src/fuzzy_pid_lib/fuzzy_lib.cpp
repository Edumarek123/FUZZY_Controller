//
// Created by Edumarek on 30/03/2023.
//

#include"PID_lib.hpp"

//------------------CONSTRUTORES&DESTRUTORES-------------------//
ControladorFuzzy::ControladorFuzzy(_ponto tA, _ponto min_uK, _ponto max_uK){
    Ta = tA;
    min_Uk = min_uK;
    max_Uk = max_uK;

    int hl[] = {10};
    RR = new RedeNeural(8, 4, hl, sizeof(hl) / sizeof(int), 1); //entradas, saidas, camadas ocultas, numero de camadas ocultas, leraning_rate
}

ControladorFuzzy::ControladorFuzzy(_ponto kP, _ponto kD, _ponto kI, _ponto n, _ponto tA, _ponto min_uK, _ponto max_uK){
    Kp = kP;
    Kd = kD;
    Ki = kI;
    Ta = tA;
    N = n;
    min_Uk = min_uK;
    max_Uk = max_uK;

    int hl[] = {10};
    RR = new RedeNeural(8, 4, hl, sizeof(hl) / sizeof(int), 1); //entradas, saidas, camadas ocultas, leraning_rate
}

ControladorFuzzy::ControladorFuzzy(const ControladorFuzzy &c){
    Kp = c.Kp;
    Kd = c.Kd;
    Ki = c.Ki;
    N = c.N;
    Ta = c.Ta;
    min_Uk = c.min_Uk;
    max_Uk = c.max_Uk;

    limiteIteracoesAcaoControle = c.limiteIteracoesAcaoControle;

    RR = c.RR;
}

ControladorFuzzy::~ControladorFuzzy(){
    delete RR;

    free(delimitadorMaxX);
    free(delimitadorMinY);
    free(delimitadorMaxY);
}

//---------------------------METODOS---------------------------//

#if defined __linux__
void ControladorFuzzy::Imprimir_Parametros(){
    std::cout<<"Kp: "<<Kp<<" | Kd: "<<Kd<<" | Ki: "<<Ki<<" | N: "<<N<<" | Ta: "<<Ta<<std::endl;

}
#else
void ControladorFuzzy::Imprimir_Parametros(){
    Serial.println("Kp: "+String((double)Kp)+" | Kd: "+String((double)Kd)+" | Ki: "+String((double)Ki)+" | N: "+String((double)N));
}
#endif

#if defined __linux__
_ponto ControladorFuzzy::Calcula_Acao_Controle(_ponto referencia, _ponto saida){
    clock_t tInicio = clock();
    double a, b ,c, d, e;

    //Calcula erro
    Yk[0] = saida;
    Ek[0]  = referencia - saida;

    //Calcula acao de controle

    //Limitador acao controle
    if(Uk[0] > max_Uk)
        Uk[0] = max_Uk;
    if(Uk[0] <= min_Uk)
        Uk[0] = min_Uk;

    //Atualiza Saidas
    Yk[1] = Yk[0];

    //Atualiza erros
    Ek[2] = Ek[1];
    Ek[1] = Ek[0];

    //Atualiza acoes de controle
    Uk[1] = Uk[0];
    Uk[2] = Uk[1];

    if(calcularTempoExecucao)
        tExecucao = (unsigned long)((1000 * (tInicio - clock())) / CLOCKS_PER_SEC);

    return Uk[0];
}

#else
_ponto ControladorFuzzy::Calcula_Acao_Controle(_ponto referencia, _ponto saida){
    unsigned long tInicio=micros();
    _ponto a, b ,c, d, e;

    //Calcula erro
    Yk[0] = saida;
    Ek[0]  = referencia - saida;

    //Calcula acao de controle
    a = Kp * (1 + (N * Ta)) + (Ki * Ta) * (1 + (N * Ta)) + (Kd * N);
    b = -(Kp * (2 + (N * Ta)) + (Ki * Ta)+(2 * Kd * N));
    c = Kp + (Kd * N);
    d = 1 + (N * Ta);
    e = -(2 + (N * Ta));

    Uk[0] = (_ponto)((-e * Uk[1]) - (1 * Uk[2]) + (a * Ek[0]) + (b * Ek[1]) + (c * Ek[2])) / d; //by mateus

    //Limitador acao controle
    if(Uk[0] > max_Uk)
        Uk[0] = max_Uk;
    if(Uk[0] <= min_Uk)
        Uk[0] = min_Uk;

    //Atualiza Saidas
    Yk[1] = Yk[0];

    //Atualiza erros
    Ek[2] = Ek[1];
    Ek[1] = Ek[0];

    //Atualiza acoes de controle
    Uk[1] = Uk[0];
    Uk[2] = Uk[1];

    if(calcularTempoExecucao)
        tExecucao = micros() - tInicio;

    return Uk[0];
}
#endif

void ControladorFuzzy::Atualiza_Parametros(){

}



