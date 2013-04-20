#ifndef KEYBOARDEVENTFILTER_H
#define KEYBOARDEVENTFILTER_H

#include <QObject>

/*!
 * \brief The KeyboardEventFilter class filters out specific keyboard events
 * for handling at a global scope
 */

class KeyboardEventFilter : public QObject
{
    Q_OBJECT
public:
    explicit KeyboardEventFilter(QObject *parent = 0);

protected:
    /*!
     * \brief Takes a QEvent and filters out specific keyboard events
     * \param obj
     * \param event The event to be filtered
     * \return true if the event was filtered out, false otherwise
     */
    bool eventFilter(QObject *obj, QEvent *event);

signals:
    /*!
     * \brief Emitted whenever and Escape key event is passed into the filter
     */
    void escapePressed();

public slots:

};

#endif // KEYBOARDEVENTFILTER_H
