#ifndef WAIT_DIALOG_H
#define WAIT_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>

class WaitDialog : public QDialog {
        Q_OBJECT

    public:
        WaitDialog(const QString& message, QWidget *parent = 0);

    public slots:
        void show();
        void hide();
        void push();
        void pop();

    signals:
        void visibilityChanged(bool enable);

    private:
        int stack;
        QLabel* msgLabel;
};


#endif // WAIT_DIALOG_H
