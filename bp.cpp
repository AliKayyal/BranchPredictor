/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */


#include <iostream>
#include "bp_api.h"
enum State{SN,WN,WT,ST};

int branches_counter=0;
int mis_counter=0;
bool last_prediction;
int size=0;
int initial_state;// static functions

static int Power(int base, int exponent) {
    int result = 1;
    for (int i = 0; i < exponent; i++)
        result *= base;
    return result;
}


class BranchFSM {
public:

    int state;
    BranchFSM():state(initial_state) {
    }
    bool Prediction() {
        return (state==ST||state==WT);
    }
    void updateState(bool check_taken) {
        if (check_taken==true) {
            if(state!=ST){
                state+=1;
            }
        }
        else {
            if(state!=SN){
                state-=1;
            }
        }
    }
    void Initialize(int st){
        state=st;
    }
    BranchFSM& operator=(unsigned fsmState){
        state=fsmState;
        initial_state=fsmState;
    }

};

/***************************************************************************************/

class Target {
public:
    uint32_t tag, target;
    bool valid;
    Target() :tag(0), target(0), valid(false) {}
};

/***************************************************************************************/

class BranchPredictor {

public:
    unsigned btb_size,tag_size, bhr_size, states_size;
    Target* btb_table;
    uint32_t tag_mask;
    virtual bool Prediction(uint32_t pc, uint32_t* dst)=0;
    virtual void updateBranchState(uint32_t pc, uint32_t target,bool check_taken, uint32_t dst)=0;
    virtual ~BranchPredictor(){};
};

class Local_Local_Predictor: public BranchPredictor {
private:
    int* history;
    BranchFSM** branch_states;
public:
    Local_Local_Predictor(int btb_size, int bhr_size, int tag_size,unsigned fsmState){
        initial_state=fsmState;
        this->btb_size=btb_size;
        this->bhr_size=bhr_size;
        this->tag_size=tag_size;
        tag_mask = Power(2, tag_size) - 1;
        states_size = Power(2, bhr_size);
        btb_table = new Target[btb_size];
        history = new int[btb_size];
        branch_states = new BranchFSM*[btb_size];
        for (int i = 0; i < btb_size; i++){
            branch_states[i] = new BranchFSM[states_size];

        }
    }

    virtual ~Local_Local_Predictor(){
        delete[] btb_table;
        delete[] history;
        for (int i = 0; i < btb_size; i++)
            delete[] branch_states[i];
        delete[] branch_states;
    }

    virtual bool Prediction(uint32_t pc, uint32_t* dst) {
        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;
        bool taken = false;

        *dst = pc + 4;
        if (!btb_table[index].valid || btb_table[index].tag != tag) {
            return false;
        }

        taken = branch_states[index][history[index]].Prediction();
        if (taken)
            *dst = btb_table[index].target;

        return taken;
    }

    virtual void updateBranchState(uint32_t pc, uint32_t target,
                                   bool check_taken, uint32_t pred_dst) {

        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;

        if (!btb_table[index].valid || btb_table[index].tag != tag) {
            btb_table[index].tag=tag;
            btb_table[index].valid=true;
            history[index]=0;
            for(int i=0; i < states_size; i++)
                branch_states[index][i].Initialize(initial_state);
        }

        btb_table[index].target=target;
        branch_states[index][history[index]].updateState(check_taken);
        history[index]=history[index] << 1;
        if (check_taken) history[index]++;
        history[index]=history[index] % states_size;
    }

};

class Local_Global_Predictor: public BranchPredictor {

private:
    int* history;
    BranchFSM* branch_states;
    int shared;

public:
    Local_Global_Predictor(int btb_size, int bhr_size, int tag_size, int Shared,unsigned fsmState):shared(Shared){
        this->btb_size=btb_size;
        this->bhr_size=bhr_size;
        this->tag_size=tag_size;
        initial_state=fsmState;
        states_size = Power(2, bhr_size);
        tag_mask = Power(2, tag_size) - 1;
        btb_table = new Target[btb_size];
        history = new int[btb_size];
        branch_states = new BranchFSM[states_size];
    }

    virtual ~Local_Global_Predictor() {
        delete[] btb_table;
        delete[] history;
        delete[] branch_states;
    }

    virtual bool Prediction(uint32_t pc, uint32_t* dst) {
        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;

        bool taken = false;
        *dst = pc + 4;
        if (!btb_table[index].valid || btb_table[index].tag != tag) {
            return false;
        }

        int fixed_history ;
        if( shared==1 ){
            fixed_history=((pc >> 2) ^ history[index]) % states_size;
        }
        else if(shared==2){
            fixed_history=((pc >> 16) ^ history[index]) % states_size;
        }
        else{
            fixed_history=history[index];
        }

        taken = branch_states[fixed_history].Prediction();
        if (taken) *dst = btb_table[index].target;

        return taken;
    }

    virtual void updateBranchState(uint32_t pc, uint32_t target,
                                   bool check_taken, uint32_t pred_dst) {

        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;

        if (!btb_table[index].valid || btb_table[index].tag != tag) {
            history[index] = 0;
        }

        btb_table[index].tag = tag;
        btb_table[index].valid = true;
        btb_table[index].target = target;

        int fixed_history ;
        if( shared==1 ){
            fixed_history=((pc >> 2) ^ history[index]) % states_size;
        }
        else if(shared==2){
            fixed_history=((pc >> 16) ^ history[index]) % states_size;
        }
        else{
            fixed_history=history[index];
        }
        branch_states[fixed_history].updateState(check_taken);
        history[index] = history[index] << 1;
        if (check_taken)
            history[index]++;
        history[index] = history[index] % states_size;
    }

};


class Global_Global_Predictor: public BranchPredictor {

private:
    BranchFSM* branch_states;
    int history;
    int shared;

public:
    Global_Global_Predictor(int btb_size, int bhr_size, int tag_size, int Shared,unsigned fsmState) {
        history=0;
        this->btb_size = btb_size;
        this->bhr_size = bhr_size;
        this->tag_size = tag_size;
        this->shared  = Shared;
        initial_state=fsmState;

        states_size = Power(2, bhr_size);
        tag_mask = Power(2, tag_size) - 1;

        btb_table = new Target[btb_size];
        branch_states = new BranchFSM[states_size];
    }

    virtual ~Global_Global_Predictor() {
        delete[] btb_table;
        delete[] branch_states;
    }

    virtual bool Prediction(uint32_t pc, uint32_t* dst) {
        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;
        bool taken = false;

        *dst = pc + 4;
        if (btb_table[index].valid== false || btb_table[index].tag != tag) {
            return false;
        }

        int fixed_history ;
        if( shared==1 ){
            fixed_history=((pc >> 2) ^ history) % states_size;
        }
        else if(shared==2){
            fixed_history=((pc >> 16) ^ history) % states_size;
        }
        else{
            fixed_history=history;
        }
        taken = branch_states[fixed_history].Prediction();
        if (taken==true)
            *dst = btb_table[index].target;

        return taken;
    }

    virtual void updateBranchState(uint32_t pc, uint32_t target,
                                   bool check_taken, uint32_t pred_dst) {

        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;

        if (!btb_table[index].valid || btb_table[index].tag != tag) {
            btb_table[index].valid = true;
        }

        btb_table[index].tag = tag;
        btb_table[index].target = target;

        int fixed_history ;
        if( shared==1 ){
            fixed_history=((pc >> 2) ^ history) % states_size;
        }
        else if(shared==2){
            fixed_history=((pc >> 16) ^ history) % states_size;
        }
        else{
            fixed_history=history;
        }
        branch_states[fixed_history].updateState(check_taken);
        history = history << 1;
        if (check_taken)
            history++;
        history = history % states_size;
    }


};

class Global_Local_Predictor: public BranchPredictor {
private:
    BranchFSM** branch_states;
    int history;
public:
    Global_Local_Predictor(int btb_size, int bhr_size, int tag_size,unsigned fsmState){
        history=0;
        this->btb_size = btb_size;
        this->bhr_size = bhr_size;
        this->tag_size = tag_size;
        initial_state=fsmState;
        states_size = Power(2, bhr_size);
        tag_mask = Power(2, tag_size) - 1;

        btb_table = new Target[btb_size];
        branch_states = new BranchFSM*[btb_size];
        for (int i = 0; i < btb_size; i++){
            branch_states[i] = new BranchFSM[states_size];
        }
    }

    virtual bool Prediction(uint32_t pc, uint32_t* dst) {
        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;
        bool taken = false;

        *dst = pc + 4;
        if (btb_table[index].valid==false || btb_table[index].tag != tag) {
            return false;
        }

        taken = branch_states[index][history].Prediction();
        if (taken)
            *dst = btb_table[index].target;

        return taken;
    }

    virtual void updateBranchState(uint32_t pc, uint32_t target,
                                   bool check_taken, uint32_t pred_dst) {

        int index = (pc >> 2) % btb_size;
        uint32_t tag = (pc >> 2) & tag_mask;

        if (!btb_table[index].valid || btb_table[index].tag != tag) {
            btb_table[index].tag=tag;
            btb_table[index].valid=true;
            for(int i=0; i < states_size; i++)
                branch_states[index][i].Initialize(initial_state);
        }

        btb_table[index].target=target;
        branch_states[index][history].updateState(check_taken);
        history=history << 1;
        if (check_taken) history++;
        history=history % states_size;
    }

    virtual ~Global_Local_Predictor(){
        delete[] btb_table;
        for (int i = 0; i < btb_size; i++)
            delete[] branch_states[i];
        delete[] branch_states;
    }
};

/***************************************************************************************/

BranchPredictor *predictor;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared){

    if(btbSize < 1 || historySize < 1 || tagSize < 0 ) return -1;

    if (isGlobalHist) {
        if (isGlobalTable) {
            size=btbSize*(tagSize+30)+historySize+2*Power(2,historySize);
            predictor = new Global_Global_Predictor(btbSize, historySize, tagSize, Shared,fsmState);
        } else {
            size=btbSize*(tagSize+30+2*Power(2,historySize))+historySize;
            predictor = new Global_Local_Predictor(btbSize, historySize, tagSize,fsmState);
        }
    } else {
        if (isGlobalTable) {
            size=btbSize*(tagSize+30+historySize)+2*Power(2,historySize);
            predictor = new Local_Global_Predictor(btbSize, historySize, tagSize, Shared,fsmState);
        } else {
            size=btbSize*(tagSize+30+historySize+2*Power(2,historySize));
            predictor = new Local_Local_Predictor(btbSize, historySize, tagSize,fsmState);
        }
    }
    return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst) {
    last_prediction=predictor->Prediction(pc, dst);
    return last_prediction;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    branches_counter++;
    uint32_t actually=taken ? targetPc : pc+4;
    if (actually != pred_dst)
        mis_counter++;

    predictor->updateBranchState(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats) {
    curStats->br_num    = branches_counter;
    curStats->flush_num = mis_counter;
    curStats->size=size;
    return;
}
//
// Created by jamal on 17/04/2019.
//











