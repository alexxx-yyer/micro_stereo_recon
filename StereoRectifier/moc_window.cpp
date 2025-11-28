/****************************************************************************
** Meta object code from reading C++ file 'window.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "window.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'window.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_window_t {
    QByteArrayData data[14];
    char stringdata0[267];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_window_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_window_t qt_meta_stringdata_window = {
    {
QT_MOC_LITERAL(0, 0, 6), // "window"
QT_MOC_LITERAL(1, 7, 21), // "on_loadParams_clicked"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 7), // "checked"
QT_MOC_LITERAL(4, 38, 21), // "on_loadImages_clicked"
QT_MOC_LITERAL(5, 60, 20), // "on_outputDir_clicked"
QT_MOC_LITERAL(6, 81, 18), // "on_rectify_clicked"
QT_MOC_LITERAL(7, 100, 27), // "on_imgWidth_editingFinished"
QT_MOC_LITERAL(8, 128, 28), // "on_imgHeight_editingFinished"
QT_MOC_LITERAL(9, 157, 24), // "on_showRectified_toggled"
QT_MOC_LITERAL(10, 182, 21), // "on_checkAlpha_toggled"
QT_MOC_LITERAL(11, 204, 34), // "on_presetSizes_currentIndexCh..."
QT_MOC_LITERAL(12, 239, 5), // "index"
QT_MOC_LITERAL(13, 245, 21) // "on_saveParams_clicked"

    },
    "window\0on_loadParams_clicked\0\0checked\0"
    "on_loadImages_clicked\0on_outputDir_clicked\0"
    "on_rectify_clicked\0on_imgWidth_editingFinished\0"
    "on_imgHeight_editingFinished\0"
    "on_showRectified_toggled\0on_checkAlpha_toggled\0"
    "on_presetSizes_currentIndexChanged\0"
    "index\0on_saveParams_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_window[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   64,    2, 0x08 /* Private */,
       4,    1,   67,    2, 0x08 /* Private */,
       5,    1,   70,    2, 0x08 /* Private */,
       6,    1,   73,    2, 0x08 /* Private */,
       7,    0,   76,    2, 0x08 /* Private */,
       8,    0,   77,    2, 0x08 /* Private */,
       9,    1,   78,    2, 0x08 /* Private */,
      10,    1,   81,    2, 0x08 /* Private */,
      11,    1,   84,    2, 0x08 /* Private */,
      13,    1,   87,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Int,   12,
    QMetaType::Void, QMetaType::Bool,    3,

       0        // eod
};

void window::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<window *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_loadParams_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_loadImages_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_outputDir_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_rectify_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_imgWidth_editingFinished(); break;
        case 5: _t->on_imgHeight_editingFinished(); break;
        case 6: _t->on_showRectified_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_checkAlpha_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_presetSizes_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_saveParams_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject window::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_window.data,
    qt_meta_data_window,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *window::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *window::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_window.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int window::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
