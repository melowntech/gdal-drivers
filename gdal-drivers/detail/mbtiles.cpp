#include <cstring>

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <sqlite3.h>

#include <cpl_error.h>

#include "dbglog/dbglog.hpp"

#include "./mbtiles.hpp"

namespace bio = boost::iostreams;

namespace gdal_drivers { namespace detail {

namespace {

struct Database {
    Database() : db() {}
    ~Database() { if (db) { ::sqlite3_close(db); } }
    operator ::sqlite3*() { return db; }
    ::sqlite3 *db;
};

struct Statement {
    Statement() : stmt() {}
    ~Statement() { if (stmt) { ::sqlite3_finalize(stmt); } }
    operator ::sqlite3_stmt*() { return stmt; }
    ::sqlite3_stmt *stmt;
};

inline bool isDigit(char c) { return (c >= '0') && (c <= '9'); }

inline char positive(char c) { return c - '0'; }

template <unsigned int minWidth, char(*getter)(char), typename T>
inline const char* parsePartImpl(const char *p, T &value)
{
    bool prefix = false;
    char c(p[0]);
    switch (c) {
    case '-': case '+': return nullptr;
    case '0': prefix = true;
    }

    value = 0;

    const char *e(p);
    while (isDigit(c)) {
        value *= 10;
        value += getter(c);
        c = *++e;
    }

    auto dist(e - p);
    if (dist < minWidth) { return nullptr; }
    if (prefix && (dist > minWidth)) { return nullptr; }
    return e;
}

template <unsigned int minWidth, typename T>
inline const char* parsePart(const char *p, T &value)
{
    // only positive numbers are allowed
    return parsePartImpl<minWidth, positive>(p, value);
}

bool parse(const char *p, int &zoom, int &col, int &row)
{
    if (!(p = parsePart<1>(p, zoom))) { return false; }
    if (*p++ != '-') { return false; }

    if (!(p = parsePart<1>(p, col))) { return false; }
    if (*p++ != '-') { return false; }

    if (!(p = parsePart<1>(p, row))) { return false; }
    return !*p;
}

bool check(int status, Database &db, const char *what)
{
    if (!status) { return false; }
    const char *msg(::sqlite3_errmsg(db));
    LOG(err2) << "Sqlite3 operation " << what << " failed: <"
              << msg << ">.";

    ::CPLError(CE_Failure, CPLE_AppDefined
               , "Sqlite3 operation %s failed: <%s>.", what, msg);

    return true;
}

} // namespace

bool loadFromMbTilesArchive(vector_tile::Tile &tile, const char *path)
{
    // last slash
    const auto *p(::strrchr(path, '/'));
    if (!p) {
        ::CPLError(CE_Failure, CPLE_AppDefined
                   , "Cannot find slash in path <%s>.", path);
        return false;
    }

    // try to parse zoom-row-col
    int zoom(0), col(0), row(0);
    if (!parse(p + 1, zoom, col, row)) {
        ::CPLError(CE_Failure, CPLE_AppDefined
                   , "Unable to match zoom-col-row in the last element"
                   " of <%s>.", path);
    }

    LOG(info4) << "zoom: " << zoom << ", col: " << col << ", row: " << row;

    const std::string mbtiles(path, p);

    // open database
    Database db;
    if (check(::sqlite3_open_v2(mbtiles.c_str(), &db.db, SQLITE_OPEN_READONLY
                                , nullptr)
              , db, "sqlite3_open_v2"))  { return false; }

    // prepare statement
    Statement stmt;
    if (check(::sqlite3_prepare(db
                                , ("SELECT tile_data "
                                   "FROM tiles WHERE zoom_level=?"
                                   "     AND tile_column=?"
                                   "     AND tile_row=?")
                                , -1 // read until \0
                                , &stmt.stmt
                                , nullptr)
              , db, "sqlite3_prepare")) { return false; }

    // bind
    if (check(::sqlite3_bind_int(stmt, 1, zoom), db, "sqlite3_bind_int"))
        { return false; }
    if (check(::sqlite3_bind_int(stmt, 2, col), db, "sqlite3_bind_int"))
        { return false; }
    if (check(::sqlite3_bind_int(stmt, 3, row), db, "sqlite3_bind_int"))
        { return false; }

    switch (auto res = ::sqlite3_step(stmt)) {
    case SQLITE_ROW: break;

    case SQLITE_DONE:
        ::CPLError(CE_Failure, CPLE_AppDefined
                   , "No tile %d-%d-%d found in database file <%s>."
                   , zoom, col, row, mbtiles.c_str());
        return false;

    default:
        check(res, db, "sqlite3_step");
        return false;
    }

    // get blob
    const char *blob(static_cast<const char*>(::sqlite3_column_blob(stmt, 0)));
    if (!blob) {
        ::CPLError(CE_Failure, CPLE_AppDefined
                   , "Unable to get blob from query result (%s)", path);
        return false;
    }
    const auto blobSize(::sqlite3_column_bytes(stmt, 0));

    if (!blobSize) {
        ::CPLError(CE_Failure, CPLE_AppDefined
                   , "Empty blob in query result (%s)", path);
        return false;
    }

    if (*blob != 0x1f) {
        // probably not gzipped -> return as is
        return tile.ParseFromArray(blob, blobSize);
    };

    // gunzip and decode
    bio::stream_buffer<bio::array_source> buffer(blob, blob + blobSize);
    bio::filtering_istream gzipped;
    gzipped.push(bio::gzip_decompressor());
    gzipped.push(buffer);
    return tile.ParseFromIstream(&gzipped);
}

} } // namespace gdal_drivers::detail
