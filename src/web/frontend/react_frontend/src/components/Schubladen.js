import PropTypes from 'prop-types'
import Schublade from './Schublade'


const Schubladen = ({schubladen, onChange}) => {     
        return (
                <>
                  {
                          schubladen.map((schublade) => (<Schublade 
                                key={schublade.id} 
                                schublade={schublade} 
                                onChange = {onChange} 
                               />))
                  }      
                </>
        )
}


Schubladen.protoTypes = {
        schubladen: PropTypes.object,
        onChange: PropTypes.func,
}

export default Schubladen
