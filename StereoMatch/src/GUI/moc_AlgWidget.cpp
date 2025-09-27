/****************************************************************************
** Meta object code from reading C++ file 'AlgWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "AlgWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'AlgWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_AlgWidget_t {
    QByteArrayData data[1];
    char stringdata0[10];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AlgWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AlgWidget_t qt_meta_stringdata_AlgWidget = {
    {
QT_MOC_LITERAL(0, 0, 9) // "AlgWidget"

    },
    "AlgWidget"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AlgWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void AlgWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject AlgWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_AlgWidget.data,
    qt_meta_data_AlgWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *AlgWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AlgWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AlgWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int AlgWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_sgbm_setting_t {
    QByteArrayData data[15];
    char stringdata0[303];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_sgbm_setting_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_sgbm_setting_t qt_meta_stringdata_sgbm_setting = {
    {
QT_MOC_LITERAL(0, 0, 12), // "sgbm_setting"
QT_MOC_LITERAL(1, 13, 25), // "on_BlockSize_valueChanged"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 3), // "val"
QT_MOC_LITERAL(4, 44, 23), // "on_numDisp_valueChanged"
QT_MOC_LITERAL(5, 68, 23), // "on_minDisp_valueChanged"
QT_MOC_LITERAL(6, 92, 18), // "on_p1_valueChanged"
QT_MOC_LITERAL(7, 111, 18), // "on_p2_valueChanged"
QT_MOC_LITERAL(8, 130, 28), // "on_preFilterCap_valueChanged"
QT_MOC_LITERAL(9, 159, 27), // "on_uniqueRatio_valueChanged"
QT_MOC_LITERAL(10, 187, 27), // "on_speckleSize_valueChanged"
QT_MOC_LITERAL(11, 215, 28), // "on_speckleRange_valueChanged"
QT_MOC_LITERAL(12, 244, 29), // "on_disp12MaxDiff_valueChanged"
QT_MOC_LITERAL(13, 274, 24), // "on_modeList_IndexChanged"
QT_MOC_LITERAL(14, 299, 3) // "idx"

    },
    "sgbm_setting\0on_BlockSize_valueChanged\0"
    "\0val\0on_numDisp_valueChanged\0"
    "on_minDisp_valueChanged\0on_p1_valueChanged\0"
    "on_p2_valueChanged\0on_preFilterCap_valueChanged\0"
    "on_uniqueRatio_valueChanged\0"
    "on_speckleSize_valueChanged\0"
    "on_speckleRange_valueChanged\0"
    "on_disp12MaxDiff_valueChanged\0"
    "on_modeList_IndexChanged\0idx"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_sgbm_setting[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x08 /* Private */,
       4,    1,   72,    2, 0x08 /* Private */,
       5,    1,   75,    2, 0x08 /* Private */,
       6,    1,   78,    2, 0x08 /* Private */,
       7,    1,   81,    2, 0x08 /* Private */,
       8,    1,   84,    2, 0x08 /* Private */,
       9,    1,   87,    2, 0x08 /* Private */,
      10,    1,   90,    2, 0x08 /* Private */,
      11,    1,   93,    2, 0x08 /* Private */,
      12,    1,   96,    2, 0x08 /* Private */,
      13,    1,   99,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,   14,

       0        // eod
};

void sgbm_setting::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<sgbm_setting *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_BlockSize_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->on_numDisp_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_minDisp_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->on_p1_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->on_p2_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->on_preFilterCap_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_uniqueRatio_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_speckleSize_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_speckleRange_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_disp12MaxDiff_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_modeList_IndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject sgbm_setting::staticMetaObject = { {
    QMetaObject::SuperData::link<AlgWidget::staticMetaObject>(),
    qt_meta_stringdata_sgbm_setting.data,
    qt_meta_data_sgbm_setting,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *sgbm_setting::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *sgbm_setting::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_sgbm_setting.stringdata0))
        return static_cast<void*>(this);
    return AlgWidget::qt_metacast(_clname);
}

int sgbm_setting::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = AlgWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
struct qt_meta_stringdata_bm_setting_t {
    QByteArrayData data[14];
    char stringdata0[274];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_bm_setting_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_bm_setting_t qt_meta_stringdata_bm_setting = {
    {
QT_MOC_LITERAL(0, 0, 10), // "bm_setting"
QT_MOC_LITERAL(1, 11, 20), // "on_BlockSize_changed"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 3), // "val"
QT_MOC_LITERAL(4, 37, 18), // "on_numDisp_changed"
QT_MOC_LITERAL(5, 56, 18), // "on_minDisp_changed"
QT_MOC_LITERAL(6, 75, 23), // "on_preFilterCap_changed"
QT_MOC_LITERAL(7, 99, 24), // "on_preFilterSize_changed"
QT_MOC_LITERAL(8, 124, 24), // "on_preFilterType_changed"
QT_MOC_LITERAL(9, 149, 23), // "on_SpeckleRange_changed"
QT_MOC_LITERAL(10, 173, 22), // "on_SpeckleSize_changed"
QT_MOC_LITERAL(11, 196, 24), // "on_uniquessRatio_changed"
QT_MOC_LITERAL(12, 221, 27), // "on_textureThreshold_changed"
QT_MOC_LITERAL(13, 249, 24) // "on_disp12MaxDiff_changed"

    },
    "bm_setting\0on_BlockSize_changed\0\0val\0"
    "on_numDisp_changed\0on_minDisp_changed\0"
    "on_preFilterCap_changed\0"
    "on_preFilterSize_changed\0"
    "on_preFilterType_changed\0"
    "on_SpeckleRange_changed\0on_SpeckleSize_changed\0"
    "on_uniquessRatio_changed\0"
    "on_textureThreshold_changed\0"
    "on_disp12MaxDiff_changed"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_bm_setting[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x08 /* Private */,
       4,    1,   72,    2, 0x08 /* Private */,
       5,    1,   75,    2, 0x08 /* Private */,
       6,    1,   78,    2, 0x08 /* Private */,
       7,    1,   81,    2, 0x08 /* Private */,
       8,    1,   84,    2, 0x08 /* Private */,
       9,    1,   87,    2, 0x08 /* Private */,
      10,    1,   90,    2, 0x08 /* Private */,
      11,    1,   93,    2, 0x08 /* Private */,
      12,    1,   96,    2, 0x08 /* Private */,
      13,    1,   99,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,

       0        // eod
};

void bm_setting::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<bm_setting *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_BlockSize_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->on_numDisp_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_minDisp_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->on_preFilterCap_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->on_preFilterSize_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->on_preFilterType_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_SpeckleRange_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_SpeckleSize_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_uniquessRatio_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_textureThreshold_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_disp12MaxDiff_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject bm_setting::staticMetaObject = { {
    QMetaObject::SuperData::link<AlgWidget::staticMetaObject>(),
    qt_meta_stringdata_bm_setting.data,
    qt_meta_data_bm_setting,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *bm_setting::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *bm_setting::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_bm_setting.stringdata0))
        return static_cast<void*>(this);
    return AlgWidget::qt_metacast(_clname);
}

int bm_setting::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = AlgWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
