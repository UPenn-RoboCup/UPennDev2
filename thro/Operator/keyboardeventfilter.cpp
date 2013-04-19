#include "keyboardeventfilter.h"
#include <QEvent>
#include <QKeyEvent>

KeyboardEventFilter::KeyboardEventFilter(QObject *parent) :
    QObject(parent)
{
}

bool KeyboardEventFilter::eventFilter(QObject* obj, QEvent* event)
{
    // Filter out only KeyPress events
    if(event->type() == QEvent::KeyPress) {

        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

        // Handle specific KeyEvents
        switch(keyEvent->key())
        {
        case Qt::Key_Escape:
            emit escapePressed();
            return true;
            break;

        // TODO: Handle any other globally-captured key events

        default:
            // Pass key event down to child objects/widgets
            return QObject::eventFilter(obj, event);
        }

    } else {
        // Pass other events down to child objects/widgets
        return QObject::eventFilter(obj, event);
    }
}
