package my.ADR;

import java.util.Arrays;
import javax.swing.ImageIcon;

public class Passcode extends javax.swing.JPanel {
    ImageIcon btnPress = new ImageIcon(getClass().getResource("Images/BtnPress.png"));
    ImageIcon btnRelease = new ImageIcon(getClass().getResource("Images/BtnOutline.png"));
    
    public int pcCount = 0;
    public char[] userPC = new char[4];
    public String userPasscode;
    
    public Passcode() {
        initComponents();
        
        enter.setEnabled(false);
        delete.setEnabled(false);
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        nmbr0 = new javax.swing.JLabel();
        nmbr1 = new javax.swing.JLabel();
        nmbr2 = new javax.swing.JLabel();
        nmbr3 = new javax.swing.JLabel();
        nmbr4 = new javax.swing.JLabel();
        nmbr5 = new javax.swing.JLabel();
        nmbr6 = new javax.swing.JLabel();
        nmbr7 = new javax.swing.JLabel();
        nmbr8 = new javax.swing.JLabel();
        nmbr9 = new javax.swing.JLabel();
        delete = new javax.swing.JLabel();
        enter = new javax.swing.JLabel();
        msgLabel = new javax.swing.JLabel();
        answrLabel = new javax.swing.JLabel();

        setLayout(new org.netbeans.lib.awtextra.AbsoluteLayout());

        nmbr0.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr0.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr0.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr0.setText("0");
        nmbr0.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr0.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr0MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr0MouseReleased(evt);
            }
        });
        add(nmbr0, new org.netbeans.lib.awtextra.AbsoluteConstraints(570, 350, -1, -1));

        nmbr1.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr1.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr1.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr1.setText("1");
        nmbr1.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr1.setMaximumSize(new java.awt.Dimension(100, 100));
        nmbr1.setMinimumSize(new java.awt.Dimension(30, 30));
        nmbr1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr1MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr1MouseReleased(evt);
            }
        });
        add(nmbr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(460, 95, -1, -1));

        nmbr2.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr2.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr2.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr2.setText("2");
        nmbr2.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr2.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr2MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr2MouseReleased(evt);
            }
        });
        add(nmbr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(570, 95, -1, -1));

        nmbr3.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr3.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr3.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr3.setText("3");
        nmbr3.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr3.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr3MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr3MouseReleased(evt);
            }
        });
        add(nmbr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(680, 95, -1, -1));

        nmbr4.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr4.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr4.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr4.setText("4");
        nmbr4.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr4.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr4MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr4MouseReleased(evt);
            }
        });
        add(nmbr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(460, 180, -1, -1));

        nmbr5.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr5.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr5.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr5.setText("5");
        nmbr5.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr5.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr5MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr5MouseReleased(evt);
            }
        });
        add(nmbr5, new org.netbeans.lib.awtextra.AbsoluteConstraints(570, 180, -1, -1));

        nmbr6.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr6.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr6.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr6.setText("6");
        nmbr6.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr6.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr6MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr6MouseReleased(evt);
            }
        });
        add(nmbr6, new org.netbeans.lib.awtextra.AbsoluteConstraints(680, 180, -1, -1));

        nmbr7.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr7.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr7.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr7.setText("7");
        nmbr7.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr7.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr7MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr7MouseReleased(evt);
            }
        });
        add(nmbr7, new org.netbeans.lib.awtextra.AbsoluteConstraints(460, 265, -1, -1));

        nmbr8.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr8.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr8.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr8.setText("8");
        nmbr8.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr8.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr8MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr8MouseReleased(evt);
            }
        });
        add(nmbr8, new org.netbeans.lib.awtextra.AbsoluteConstraints(570, 265, -1, -1));

        nmbr9.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 36)); // NOI18N
        nmbr9.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nmbr9.setIcon(new javax.swing.ImageIcon(getClass().getResource("/my/ADR/Images/BtnOutline.png"))); // NOI18N
        nmbr9.setText("9");
        nmbr9.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        nmbr9.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                nmbr9MousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                nmbr9MouseReleased(evt);
            }
        });
        add(nmbr9, new org.netbeans.lib.awtextra.AbsoluteConstraints(680, 265, -1, -1));

        delete.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 24)); // NOI18N
        delete.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        delete.setText("delete");
        delete.setPreferredSize(new java.awt.Dimension(75, 75));
        delete.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                deleteMousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                deleteMouseReleased(evt);
            }
        });
        add(delete, new org.netbeans.lib.awtextra.AbsoluteConstraints(460, 350, -1, -1));

        enter.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 24)); // NOI18N
        enter.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        enter.setText("enter");
        enter.setPreferredSize(new java.awt.Dimension(75, 75));
        enter.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                enterMousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                enterMouseReleased(evt);
            }
        });
        add(enter, new org.netbeans.lib.awtextra.AbsoluteConstraints(680, 350, -1, -1));

        msgLabel.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 48)); // NOI18N
        msgLabel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        msgLabel.setText("<html><center>Please enter your passcode:</center></html>");
        msgLabel.setOpaque(true);
        msgLabel.setPreferredSize(new java.awt.Dimension(670, 65));
        add(msgLabel, new org.netbeans.lib.awtextra.AbsoluteConstraints(60, 180, 370, 130));

        answrLabel.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 44)); // NOI18N
        answrLabel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        add(answrLabel, new org.netbeans.lib.awtextra.AbsoluteConstraints(455, 10, 310, 75));
    }// </editor-fold>//GEN-END:initComponents

    private void nmbr0MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr0MousePressed
        nmbr0.setIcon(btnPress);
    }//GEN-LAST:event_nmbr0MousePressed

    private void nmbr0MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr0MouseReleased
        nmbr0.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {            
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '0';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr0MouseReleased

    private void nmbr1MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr1MousePressed
        nmbr1.setIcon(btnPress);
    }//GEN-LAST:event_nmbr1MousePressed

    private void nmbr1MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr1MouseReleased
        nmbr1.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {         
           if(pcCount == 0)
                delete.setEnabled(true);
           
            userPC[pcCount] = '1';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr1MouseReleased

    private void nmbr2MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr2MousePressed
        nmbr2.setIcon(btnPress);
    }//GEN-LAST:event_nmbr2MousePressed

    private void nmbr2MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr2MouseReleased
        nmbr2.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {       
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '2';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr2MouseReleased

    private void nmbr3MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr3MousePressed
        nmbr3.setIcon(btnPress);
    }//GEN-LAST:event_nmbr3MousePressed

    private void nmbr3MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr3MouseReleased
        nmbr3.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {      
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '3';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr3MouseReleased

    private void nmbr4MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr4MousePressed
        nmbr4.setIcon(btnPress);
    }//GEN-LAST:event_nmbr4MousePressed

    private void nmbr4MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr4MouseReleased
        nmbr4.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {        
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '4';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr4MouseReleased

    private void nmbr5MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr5MousePressed
        nmbr5.setIcon(btnPress);
    }//GEN-LAST:event_nmbr5MousePressed

    private void nmbr5MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr5MouseReleased
        nmbr5.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {          
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '5';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr5MouseReleased

    private void nmbr6MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr6MousePressed
        nmbr6.setIcon(btnPress);
    }//GEN-LAST:event_nmbr6MousePressed

    private void nmbr6MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr6MouseReleased
        nmbr6.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {       
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '6';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr6MouseReleased

    private void nmbr7MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr7MousePressed
        nmbr7.setIcon(btnPress);
    }//GEN-LAST:event_nmbr7MousePressed

    private void nmbr7MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr7MouseReleased
        nmbr7.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {            
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '7';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr7MouseReleased

    private void nmbr8MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr8MousePressed
        nmbr8.setIcon(btnPress);
    }//GEN-LAST:event_nmbr8MousePressed

    private void nmbr8MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr8MouseReleased
        nmbr8.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {            
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '8';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr8MouseReleased

    private void nmbr9MousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr9MousePressed
        nmbr9.setIcon(btnPress);
    }//GEN-LAST:event_nmbr9MousePressed

    private void nmbr9MouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_nmbr9MouseReleased
        nmbr9.setIcon(btnRelease);
        
        if(pcCount < 4)    // Collect room #
        {            
            if(pcCount == 0)
                delete.setEnabled(true);
            
            userPC[pcCount] = '9';
            pcCount++;
            
            userPasscode = new String(userPC);
            
            if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
            else
                answrLabel.setText(userPasscode);
            
            if(pcCount == 4)
                enter.setEnabled(true);
        }
    }//GEN-LAST:event_nmbr9MouseReleased

    private void deleteMousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_deleteMousePressed
        // TODO add your handling code here:
    }//GEN-LAST:event_deleteMousePressed

    private void deleteMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_deleteMouseReleased
        if(pcCount == 1)
            delete.setEnabled(false);
        else if(pcCount == 1)
            enter.setEnabled(false);
        
        if(pcCount > 0)
        {
            pcCount--;

            if(pcCount == 0)
                delete.setEnabled(false);
            else if(pcCount == 3)
                enter.setEnabled(false);
            
            userPC[pcCount] = '\0';
            userPasscode = new String(userPC);
            
            if(pcCount == 0)
                answrLabel.setText("");
            else if(pcCount < 4)
                answrLabel.setText(userPasscode + "-");
        }
    }//GEN-LAST:event_deleteMouseReleased

    private void enterMousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_enterMousePressed
        // TODO add your handling code here:
    }//GEN-LAST:event_enterMousePressed

    private void enterMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_enterMouseReleased
        if(!userPasscode.equals(MainFrame.loadPanel.pc) && pcCount == 4)    // Verifies correct room # length
        {           
            pcCount = 0;
            
            Arrays.fill(userPC, 0, 4, '\0');
            userPasscode = "";
        
            answrLabel.setText("");
            enter.setEnabled(false);
            delete.setEnabled(false);
        }
        else if(userPasscode.equals(MainFrame.loadPanel.pc) && pcCount == 4)
        {
            MainFrame.arrivalPanel.helloLabel.setText("");
            MainFrame.arrivalPanel.deliverLabel.setText("Here is your delivery!");
            MainFrame.arrivalPanel.items.setText("Items retrieved");
            
            MainFrame.passcodePanel.setVisible(false);
            MainFrame.arrivalPanel.setVisible(true);
        }
    }//GEN-LAST:event_enterMouseReleased


    // Variables declaration - do not modify//GEN-BEGIN:variables
    public javax.swing.JLabel answrLabel;
    public javax.swing.JLabel delete;
    public javax.swing.JLabel enter;
    private javax.swing.JLabel msgLabel;
    private javax.swing.JLabel nmbr0;
    private javax.swing.JLabel nmbr1;
    private javax.swing.JLabel nmbr2;
    private javax.swing.JLabel nmbr3;
    private javax.swing.JLabel nmbr4;
    private javax.swing.JLabel nmbr5;
    private javax.swing.JLabel nmbr6;
    private javax.swing.JLabel nmbr7;
    private javax.swing.JLabel nmbr8;
    private javax.swing.JLabel nmbr9;
    // End of variables declaration//GEN-END:variables
}
