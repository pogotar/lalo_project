# lalo_project

# 1

abbiamo messo l'equilibrio di 5000 m e pitch rate (128 ) a 0 perchè LQ manda lo stato a 0
e  non vogliamo calcolare l'U_bar (valore del controllo all' inf) 
(non siamo a tempo discreto dove il guadagno lo calcolo con la G(1) ma a tempo continuo e dovrei fare il lim per s ->0)

non vogliamo oscillazioni dell' altezza, se aumentiamo Q per dirgli di raggiungere l'e velocemente a un certo pnto non migliora la situa
per le saturazioni (il controllo farà sempre il massimo pox)
quindi si aumenta la R coosì da tenere sotto controllo la u prima che raggiunga le saturazioni

slew rate ??????
