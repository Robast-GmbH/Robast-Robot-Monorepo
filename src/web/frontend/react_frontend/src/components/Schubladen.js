import PropTypes from 'prop-types'
import Schublade from './Schublade'


const Schubladen = ({schubladen, onChange}) => {     
        return (
                <>
                  {
                          schubladen.Sort((a,b)=>a.index-b.index).map((schublade) => (<Schublade 
                                key={schublade.index} 
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
