/****************************************************************************
** Meta object code from reading C++ file 'panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_panel_t {
    QByteArrayData data[22];
    char stringdata0[259];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_panel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_panel_t qt_meta_stringdata_panel = {
    {
QT_MOC_LITERAL(0, 0, 5), // "panel"
QT_MOC_LITERAL(1, 6, 11), // "on_loadlist"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 7), // "checked"
QT_MOC_LITERAL(4, 27, 11), // "on_loadfile"
QT_MOC_LITERAL(5, 39, 12), // "loadlistfile"
QT_MOC_LITERAL(6, 52, 4), // "path"
QT_MOC_LITERAL(7, 57, 12), // "loadleftfile"
QT_MOC_LITERAL(8, 70, 13), // "loadrightfile"
QT_MOC_LITERAL(9, 84, 11), // "loadQMatrix"
QT_MOC_LITERAL(10, 96, 5), // "match"
QT_MOC_LITERAL(11, 102, 19), // "on_previous_clicked"
QT_MOC_LITERAL(12, 122, 15), // "on_next_clicked"
QT_MOC_LITERAL(13, 138, 16), // "on_cloud_clicked"
QT_MOC_LITERAL(14, 155, 15), // "on_save_clicked"
QT_MOC_LITERAL(15, 171, 18), // "selectDispSavePath"
QT_MOC_LITERAL(16, 190, 19), // "selectCloudSavePath"
QT_MOC_LITERAL(17, 210, 20), // "on_saveCloud_checked"
QT_MOC_LITERAL(18, 231, 5), // "state"
QT_MOC_LITERAL(19, 237, 7), // "on_sgbm"
QT_MOC_LITERAL(20, 245, 5), // "on_bm"
QT_MOC_LITERAL(21, 251, 7) // "on_elas"

    },
    "panel\0on_loadlist\0\0checked\0on_loadfile\0"
    "loadlistfile\0path\0loadleftfile\0"
    "loadrightfile\0loadQMatrix\0match\0"
    "on_previous_clicked\0on_next_clicked\0"
    "on_cloud_clicked\0on_save_clicked\0"
    "selectDispSavePath\0selectCloudSavePath\0"
    "on_saveCloud_checked\0state\0on_sgbm\0"
    "on_bm\0on_elas"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_panel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   99,    2, 0x0a /* Public */,
       4,    1,  102,    2, 0x0a /* Public */,
       5,    1,  105,    2, 0x0a /* Public */,
       7,    1,  108,    2, 0x0a /* Public */,
       8,    1,  111,    2, 0x0a /* Public */,
       9,    1,  114,    2, 0x0a /* Public */,
      10,    1,  117,    2, 0x0a /* Public */,
      11,    1,  120,    2, 0x0a /* Public */,
      12,    1,  123,    2, 0x0a /* Public */,
      13,    1,  126,    2, 0x0a /* Public */,
      14,    1,  129,    2, 0x0a /* Public */,
      15,    1,  132,    2, 0x0a /* Public */,
      16,    1,  135,    2, 0x0a /* Public */,
      17,    1,  138,    2, 0x0a /* Public */,
      19,    1,  141,    2, 0x0a /* Public */,
      20,    1,  144,    2, 0x0a /* Public */,
      21,    1,  147,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Int,   18,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,

       0        // eod
};

void panel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<panel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_loadlist((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_loadfile((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->loadlistfile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->loadleftfile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->loadrightfile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->loadQMatrix((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->match((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_previous_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_next_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->on_cloud_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->on_save_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->selectDispSavePath((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->selectCloudSavePath((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 13: _t->on_saveCloud_checked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->on_sgbm((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 15: _t->on_bm((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->on_elas((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject panel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_panel.data,
    qt_meta_data_panel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *panel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *panel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_panel.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int panel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
