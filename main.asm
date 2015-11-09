;======================================================================;
; Proyecto:	Brazo Robótico
; Nombres:	Francisco Javier Álvarez García
;		Juan Manuel Álvarez Toribio
;		Elena González Fernández
;		Rubén Haba Pascual 
; MCU:		PIC16F883
; Fecha:	28/10/2015
;======================================================================;
; Descripción:
;		Código para el control y manejo del brazo robótico.
;======================================================================;

#include <p16f883.inc>

; Reloj interno, watchdog desactivado y programación de bajo voltaje desactivada
; CONFIG1
; __config 0x2BF4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_ON & _LVP_OFF
; CONFIG2
; __config 0x3FFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF
 
;--- Direccionamiento directo para el banco 0 ---    
bank0		macro
		bcf	    status,rp0
		bcf	    status,rp1
		endm

;--- Direccionamiento directo para el banco 1 ---    
bank1		macro
		bsf	    status,rp0
		bcf	    status,rp1
		endm

;--- Direccionamiento directo para el banco 2 ---    
bank2		macro
		bcf	    status,rp0
		bsf	    status,rp1
		endm
	    
;--- Direccionamiento directo para el banco 3 ---    
bank3		macro
		bsf	    status,rp0
		bsf	    status,rp1
		endm
		
		
;-----------------;	    
;--- Variables ---;
;-----------------;
		cblock	    0x20
Sensor					    ;0x20
AuxADC					    ;0x21
ADCMux					    ;0x22
AuxVar					    ;0x23
contador				    ;0x24
		endc
		
;-------------------------;
;--- Almacen de Estado ---;
;-------------------------;
		cblock	    0x70
w_prev
status_prev
		endc

;-----------------------;
;--- Vector de reset ---;
;-----------------------;
		
		org	    0x00
resetv		goto	    init

;------------------------------;
;--- Vector de Interrupción ---;
;------------------------------;
		org	    0x04
interruptv	goto	    ISR
		
;--------------------------------;
;--- Rutina de inicialización ---;
;--------------------------------;
		org	    0x05
init		movlw	    0x70
		bank1
		movwf	    osccon	    ;8 MHz
		bank0
		call	    conf_out	    ;Configura puertos de salida
		movlw	    0x01
		movwf	    Sensor	    ;Selecciona el sensor 0
		movlw	    0x04
		movwf	    AuxADC	    ;Selecciona el sensor 2 como aux
		
		movlw	    b'01000001'	    ;Activamos el ADC
		movwf	    adcon0	    ;seleccionamos Fosc/8
		
		goto	    cap_read	    ;Lectura capacitiva
		
;--------------------------;	
;--- Lectura capacitiva ---;
;--------------------------;
		
cap_read	movfw	    Sensor	    ;Ponemos todos menos Sensor como out
		iorlw	    b'11111000'
		bank1
		movwf	    trisa	    ;Actualizamos el valor de trisa
		bank0
		movfw	    AuxADC
		movwf	    porta	    ;Actualizamos el valor de porta
		
		call	    conf_adc_mux    ;MUX ADC selecciona pin aux
		
		comf	    Sensor,w	    ;Ponemos la linea del sensor a tierra		
		bank1			    
		andwf	    trisa,f
		bank0
				
		movfw	    Sensor	    ;Ponemos el sensor como entrada
		bank1
		iorwf	    trisa,f
		bank0
		
		call	    conf_adc_mux    ;MUX ADC selecciona pin sensor
				
		movlw	    b'00000010'	    ;Comenzamos la lectura
		iorwf	    adcon0,f
		
wait		btfsc	    adcon0,go_done  ;Esperamos a que finalice
		goto	    wait
		
		comf	    Sensor,w	    
		andwf	    portc,w
		movwf	    AuxVar
		movfw	    adresh	    ;Leemos el valor analógico
		andlw	    b'11000000'	    ;Descartamos los bits menos significativos
		btfsc	    status,Z
		goto	    turn_on_led
		goto	    turn_off_led
		
turn_on_led	movfw	    AuxVar
		iorwf	    Sensor,w
		movwf	    portc
		goto	    next_adc_aux

turn_off_led	movfw	    AuxVar
		movwf	    portc
		goto	    next_adc_aux
		
;---------------------------------;
;--- Pasar al siguiente sensor ---;
;---------------------------------;
next_adc_aux	rlf	    AuxADC,f
		btfss	    AuxADC,3
		goto	    next_sensor
		movlw	    0x01
		movwf	    AuxADC
		
next_sensor	rlf	    Sensor,f
		btfss	    Sensor,3
		goto	    cap_read
		movlw	    0x01
		movwf	    Sensor
		goto	    cap_read
		
;---------------------------------;
;--- Interrupt Service Routine ---;
;---------------------------------;
ISR		movwf	    w_prev	;Almacenamos el acumulador
		swapf	    status,w	;Cargamos el registro de estado
		movwf	    status_prev	;Almacenamos el registro de estado
		
					;Cálculos necesarios
		
		swapf	    status_prev,w
		movwf	    status	;Restauramos el vector de estado
		swapf	    w_prev,f	
		swapf	    w_prev,w	;Restauramos w sin modificar status
		retfie			;Volvemos de la interrupción
		
		
;------------------;	
;--- Subrutinas ---;
;------------------;
		
;--- Configuramos el MUX ADC para que apunte al pin auxiliar ---;
conf_adc_mux	movwf	    AuxVar
		clrf	    ADCMux
calc_mux_val	rrf	    AuxVar,f
		btfsc	    status,c
		goto	    conf_mux
		incf	    ADCMux
		goto	    calc_mux_val
		
conf_mux	bcf	    status,c
		rlf	    ADCMux,f
		rlf	    ADCMux,f
		movlw	    b'11000011'
		andwf	    adcon0,w
		iorwf	    ADCMux,w
		movwf	    adcon0
		return

;--- Configura led's como salidas ---;
		
conf_out	bank1			    
		bcf	    trisc,0
		bcf	    trisc,1
		bank0
		return
		
		end


