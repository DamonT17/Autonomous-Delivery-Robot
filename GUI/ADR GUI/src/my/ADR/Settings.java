package my.ADR;

public class Settings extends javax.swing.JPanel {

    public Settings() {
        initComponents();
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        settingsLabel = new javax.swing.JLabel();
        changeLabel = new javax.swing.JLabel();
        ok = new javax.swing.JLabel();
        cancel = new javax.swing.JLabel();

        setLayout(new org.netbeans.lib.awtextra.AbsoluteLayout());

        settingsLabel.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 56)); // NOI18N
        settingsLabel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        settingsLabel.setText("Settings");
        settingsLabel.setPreferredSize(new java.awt.Dimension(200, 75));
        add(settingsLabel, new org.netbeans.lib.awtextra.AbsoluteConstraints(300, 30, -1, -1));

        changeLabel.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 40)); // NOI18N
        changeLabel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        changeLabel.setText("What would you like to change?");
        changeLabel.setPreferredSize(new java.awt.Dimension(600, 75));
        add(changeLabel, new org.netbeans.lib.awtextra.AbsoluteConstraints(100, 105, -1, -1));

        ok.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 24)); // NOI18N
        ok.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        ok.setText("ok");
        ok.setMaximumSize(new java.awt.Dimension(100, 100));
        ok.setMinimumSize(new java.awt.Dimension(30, 30));
        ok.setPreferredSize(new java.awt.Dimension(75, 75));
        ok.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                okMousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                okMouseReleased(evt);
            }
        });
        add(ok, new org.netbeans.lib.awtextra.AbsoluteConstraints(700, 390, -1, -1));

        cancel.setFont(new java.awt.Font("Nirmala UI Semilight", 0, 24)); // NOI18N
        cancel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        cancel.setText("cancel");
        cancel.setPreferredSize(new java.awt.Dimension(75, 75));
        cancel.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mousePressed(java.awt.event.MouseEvent evt) {
                cancelMousePressed(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                cancelMouseReleased(evt);
            }
        });
        add(cancel, new org.netbeans.lib.awtextra.AbsoluteConstraints(600, 390, -1, -1));
    }// </editor-fold>//GEN-END:initComponents

    private void okMousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_okMousePressed
        // TODO add your handling code here:
    }//GEN-LAST:event_okMousePressed

    private void okMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_okMouseReleased
        MainFrame.settingsPanel.setVisible(false);
        MainFrame.startPanel.setVisible(true);
    }//GEN-LAST:event_okMouseReleased

    private void cancelMousePressed(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_cancelMousePressed
        // TODO add your handling code here:
    }//GEN-LAST:event_cancelMousePressed

    private void cancelMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_cancelMouseReleased
        MainFrame.settingsPanel.setVisible(false);
        MainFrame.startPanel.setVisible(true);
    }//GEN-LAST:event_cancelMouseReleased


    // Variables declaration - do not modify//GEN-BEGIN:variables
    public javax.swing.JLabel cancel;
    private javax.swing.JLabel changeLabel;
    private javax.swing.JLabel ok;
    private javax.swing.JLabel settingsLabel;
    // End of variables declaration//GEN-END:variables
}
