#n_pagesize       :00000001  w_lcnt           :00000005  r_lcnt           :00000007  rw_hcnt          :00000003  
#frb_wait         :00000003  cmd1_wait        :00000003  addr_wait        :00000003  cmd2_wait        :00000003  
#wait_ready_wait  :00000003  addr_l           :00000000  addr_h           :00000000  addr_cycle       :00000004  
#chipselect       :00000000


insmod nfc.ko n_pagesize=2 w_lcnt=5 r_lcnt=7 rw_hcnt=3 frb_wait=3 cmd1_wait=3 addr_wait=3 cmd2_wait=3 wait_ready_wait=3 addr_l=0 addr_h=0 addr_cycle=4 chipselect=0


