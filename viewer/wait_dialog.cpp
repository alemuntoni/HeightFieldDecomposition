#include "wait_dialog.h"

WaitDialog::WaitDialog(const QString &message, QWidget *parent)
    : QDialog(parent, Qt::Dialog),
      stack(0),
      msgLabel(new QLabel(message, this))
{
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->setContentsMargins(parent->width()/2, parent->depth()/2, 100, 100);
    mainLayout->addWidget(msgLabel);
    setLayout(mainLayout);
}

void WaitDialog::show() {
    QDialog::show();

    emit visibilityChanged(true);
}

void WaitDialog::hide() {
    QDialog::hide();
    emit visibilityChanged(false);
}

void WaitDialog::push() {
    stack++;
    show();

    emit visibilityChanged(true);
}

void WaitDialog::pop() {
    if (stack > 0)
        --stack;
    if (stack == 0) {
        hide();
    }
}
