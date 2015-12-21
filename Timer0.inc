;-------------------------------------------------;
;------------------- Variables -------------------;
;-------------------------------------------------;
	    cblock  0x20
init_cap
	    endc
;-------------------------------------------------;
;---------------- Inicialización -----------------;
;-------------------------------------------------;    
in_timer0   clrwdt
	    bank0
	    clrf    tmr0	    ;Ponemos timer0 como temporizador,
	    bank1		    ;seleccionamos el prescaler para el timer 
	    movlw   b'11000000'	    
	    andwf   option_reg,w
	    iorlw   b'00000010'	    ;Prescaler a 1:8
	    movwf   option_reg
	       
;---- Habilitar interrupción del timer0 ----;	    
	    bank0
	    bsf	    intcon,t0ie
	    
	    return

;-------------------------------------------------;
;------------- Rutina de interrupción ------------;
;-------------------------------------------------;	    
isr_timer   bank0
	    bsf	    init_cap,0
	    
	    bcf	    intcon,toif	    ;Borramos bandera de timer0
	    return  
