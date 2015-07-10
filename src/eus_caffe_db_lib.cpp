#include <iostream>
#include <cassert>
#include <memory>
#include <sys/stat.h>

#include <caffe/util/db.hpp>

class eus_caffe_db{
private:
  std::shared_ptr<caffe::db::DB> db_;
  std::shared_ptr<caffe::db::Transaction> txn_;
  std::shared_ptr<caffe::db::Cursor> cursor_;
  caffe::Datum datum_;

public:
  eus_caffe_db(){
  }
  ~eus_caffe_db(){
    this->close();
  }

  int open(char* db_type, char* path, int m){
    struct stat buf;
    caffe::db::Mode mode = ((m == 'w') ? caffe::db::NEW : caffe::db::READ) ;
    if (mode == caffe::db::NEW && path && stat(path, &buf)==0){
      std::cout << "unable to open file " << path << std::endl;
      return 1;
    }
    //
    if ( this->db_ ){
      this->db_.reset(caffe::db::GetDB(db_type));
    } else {
      this->db_ = std::shared_ptr<caffe::db::DB>(caffe::db::GetDB(db_type));
    }
    this->db_->Open(path, mode);
    //
    if ( mode == caffe::db::NEW ){
      if ( this->txn_ ){
	this->txn_.reset(this->db_->NewTransaction());
      } else {
	this->txn_ = std::shared_ptr<caffe::db::Transaction>(this->db_->NewTransaction());
      }
      if ( this->cursor_ ) {
	this->cursor_.reset();
	this->cursor_ = NULL;
      }
    } else { // read mode
      if ( this->cursor_ ){
	this->cursor_.reset(this->db_->NewCursor());
      } else {
	this->cursor_ = std::shared_ptr<caffe::db::Cursor>(this->db_->NewCursor());
      }
      this->read(0);
      if ( this->txn_ ) {
	this->txn_.reset();
	this->txn_ = NULL;
      }
    }
    return 0;
  }

  int close(){
    std::cout << " -- transaction close" << std::endl;
    if ( this->txn_ ) {
      this->txn_->Commit();
      this->txn_.reset();
      this->txn_ = NULL;
    }
    std::cout << " -- cursor close" << std::endl;
    if ( this->cursor_ ) {
      this->cursor_.reset();
      this->cursor_ = NULL;
    }
    std::cout << " -- db close" << std::endl;
    if ( this->db_ ) {
      this->db_->Close();
      this->db_.reset();
      this->db_ = NULL;
    }
    return 0;
  }

  int get_data_size(){
    assert(this->cursor_);
    return this->datum_.data().size();
  }

  int get_float_data_size(){
    assert(this->cursor_);
    return this->datum_.float_data_size();
  }

  int get_key_size(){
    assert(this->cursor_);
    return this->cursor_->key().size();
  }

  int get_data(char* buf){
    assert(this->cursor_);
    const std::string& val = this->datum_.data();
    for ( int i=0; i<val.size() ; i++ ){
      buf[i] = val[i];
    }
    return val.size();
  }

  int get_float_data(double* buf){
    assert(this->cursor_);
    for ( int i=0; i<this->datum_.float_data_size() ; i++ ){
      buf[i] = this->datum_.float_data(i);
    }
    return this->datum_.float_data_size();
  }

  int get_key(char* buf){
    assert(this->cursor_);
    const std::string& val = this->cursor_->key();
    for ( int i=0; i<val.size() ; i++ ){
      buf[i] = val[i];
    }
    return val.size();
  }

  int get_shape(double* ret){
    assert(this->cursor_);
    ret[0] = this->datum_.channels();
    ret[1] = this->datum_.width();
    ret[2] = this->datum_.height();
    return 0;
  }

  int dump_datum(){
    std::cout << "[" << this->cursor_->key() << ", " << this->datum_.data() << "] ";
    std::cout << this->datum_.channels() << " x "
	      << this->datum_.width() << " x "
	      << this->datum_.height() << std::endl;
    return 0;
  }

  int read(int step){
    assert(this->cursor_);
    for ( int i=0; i<step ; i++ ){
      this->cursor_->Next();
      if (!this->cursor_->valid()) {
	std::cout << " cursor seak to first" << std::endl;
	this->cursor_->SeekToFirst();
      }
    }
    this->datum_.ParseFromString(this->cursor_->value());
    return 0;
  }

  int _put(int chan, int width, int height, int label, char* key, char* data, int data_max, double* float_data, int float_data_max){
    assert(this->txn_);
    this->datum_.set_channels(chan);
    this->datum_.set_height(width);
    this->datum_.set_width(height);
    this->datum_.set_label(label);
    if ( data && data_max > 0 ){
      this->datum_.set_data(data, data_max);
    } else {
      this->datum_.clear_data();
    }
    if ( float_data && float_data_max > 0 ){
      this->datum_.clear_float_data();
      for ( int i=0 ; i<float_data_max; i++ ){
	this->datum_.add_float_data((float)float_data[i]);
      }
    } else {
      this->datum_.clear_data();
    }
    std::string datum_str;
    this->datum_.SerializeToString(&datum_str);
    this->txn_->Put(key, datum_str);
    return 0;
  }

  int put(int chan, int width, int height, int label, char* key, char* data, int data_max){
    return this->_put(chan,width,height,label,key,data,data_max,NULL,0);
  }

  int put_double(int chan, int width, int height, int label, char* key, double* data, int data_max){
    return this->_put(chan,width,height,label,key,NULL,0,data,data_max);
  }

};

std::shared_ptr<eus_caffe_db> ecd(new eus_caffe_db);

extern "C" {
  int eus_caffe_db_open(char* db_type, char* path, int mode){ return ecd->open(db_type, path, mode); }
  int eus_caffe_db_put(int chan, int width, int height, int label, char* id_str, char* data, int data_max){ return ecd->put(chan,width,height,label,id_str,data,data_max); }
  int eus_caffe_db_put_double(int chan, int width, int height, int label, char* id_str, double* data, int data_max){ return ecd->put_double(chan,width,height,label,id_str,data,data_max); }
  int eus_caffe_db_close(){ return ecd->close() ; }
  int eus_caffe_db_read(int step){ return ecd->read(step) ; }
  int eus_caffe_db_dump(){ return ecd->dump_datum() ; }
  int eus_caffe_db_get_shape(double* ret){ return ecd->get_shape(ret) ; }
  int eus_caffe_db_get_data(char* ret){ return ecd->get_data(ret) ; }
  int eus_caffe_db_get_float_data(double* ret){ return ecd->get_float_data(ret) ; }
  int eus_caffe_db_get_key(char* ret){ return ecd->get_key(ret) ; }
  int eus_caffe_db_get_data_size(){ return ecd->get_data_size() ; }
  int eus_caffe_db_get_float_data_size(){ return ecd->get_float_data_size() ; }
  int eus_caffe_db_get_key_size(){ return ecd->get_key_size() ; }
}
